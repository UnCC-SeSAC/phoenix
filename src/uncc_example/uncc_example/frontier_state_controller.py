"""Serialize mission-side START/STOP requests for Frontier Explorer."""

import time

import rclpy

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from rclpy.task import Future

from action_msgs.msg import GoalStatus, GoalStatusArray

from frontier_exploration_ros2.srv import ControlExploration


class FrontierStateController(Node):
    """Forward START/STOP commands and wait for complete Frontier shutdown."""

    def __init__(self):
        """Create the mission-facing service and Frontier service client."""
        super().__init__('frontier_state_controller')

        self.declare_parameter(
            'frontier_control_service',
            '/control_exploration',
        )
        self.declare_parameter('stop_timeout_sec', 5.0)

        self._callback_group = ReentrantCallbackGroup()
        self._stop_timeout_sec = float(
            self.get_parameter('stop_timeout_sec').value
        )

        frontier_control_service = self.get_parameter(
            'frontier_control_service'
        ).value

        self._frontier_client = self.create_client(
            ControlExploration,
            frontier_control_service,
            callback_group=self._callback_group,
        )

        self._frontier_state = None
        self._control_request_pending = False
        self._waiting_for_nav_cancel = False
        self._stop_request_time = 0.0
        self._last_nav_status_time = 0.0
        self._nav_goal_active = False
        self._stop_wait_future = None
        self._stop_timeout_timer = None

        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._nav_status_callback,
            qos_profile_action_status_default,
            callback_group=self._callback_group,
        )

        # MissionExecutor 가 START/STOP 명령을 보낼 서비스.
        self._control_service = self.create_service(
            ControlExploration,
            '~/control_exploration',
            self._control_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            'Frontier state controller ready: '
            f'{self._control_service.srv_name} -> '
            f'{frontier_control_service}'
        )

    async def _control_callback(self, request, response):
        if request.action not in (
            ControlExploration.Request.ACTION_START,
            ControlExploration.Request.ACTION_STOP,
        ):
            return self._reject_response(
                response,
                'Unsupported control action',
            )

        if self._control_request_pending:
            return self._reject_response(
                response,
                'Another Frontier control request is in progress',
                self._current_response_state(),
            )

        if not self._frontier_client.service_is_ready():
            message = 'Frontier Explorer control service is not ready'
            self.get_logger().warn(message)
            return self._reject_response(response, message)

        self._control_request_pending = True

        try:
            if request.action == ControlExploration.Request.ACTION_START:
                return await self._handle_start(request, response)

            return await self._handle_stop(request, response)

        finally:
            self._control_request_pending = False

    async def _handle_start(self, request, response):
        if self._waiting_for_nav_cancel:
            return self._reject_response(
                response,
                'Frontier Explorer is still stopping',
                ControlExploration.Request.STATE_STOPPING,
            )

        frontier_request = ControlExploration.Request()
        frontier_request.action = ControlExploration.Request.ACTION_START
        frontier_request.delay_seconds = request.delay_seconds
        frontier_request.quit_after_stop = False

        self.get_logger().info('Forwarding ACTION_START to Frontier Explorer')

        try:
            frontier_response = await self._frontier_client.call_async(
                frontier_request
            )
        except Exception as exc:
            message = f'Frontier Explorer START failed: {exc}'
            self.get_logger().error(message)
            return self._reject_response(response, message)

        self._copy_response(frontier_response, response)
        self._frontier_state = frontier_response.state

        self._log_control_result('START', response)
        return response

    async def _handle_stop(self, request, response):
        frontier_request = ControlExploration.Request()
        frontier_request.action = ControlExploration.Request.ACTION_STOP
        frontier_request.delay_seconds = request.delay_seconds
        frontier_request.quit_after_stop = request.quit_after_stop

        self.get_logger().info('Forwarding ACTION_STOP to Frontier Explorer')

        # STOP 요청 이후에 관측된 Nav2 상태만 취소 완료 판단에 쓴다.
        # 요청 시각을 서비스 호출 전에 저장해야, 서비스 응답보다 먼저
        # 도착하는 빠른 CANCELED 상태도 놓치지 않는다.
        self._stop_request_time = time.monotonic()

        try:
            frontier_response = await self._frontier_client.call_async(
                frontier_request
            )
        except Exception as exc:
            message = f'Frontier Explorer STOP failed: {exc}'
            self.get_logger().error(message)
            return self._reject_response(response, message)

        self._copy_response(frontier_response, response)
        self._frontier_state = frontier_response.state

        if not frontier_response.accepted:
            self._log_control_result('STOP', response)
            return response

        if frontier_response.state != ControlExploration.Request.STATE_STOPPING:
            self._log_control_result('STOP', response)
            return response

        stopped = await self._wait_for_nav_cancel()

        if not stopped:
            response.accepted = False
            response.scheduled = False
            response.state = ControlExploration.Request.STATE_STOPPING
            response.message = (
                'Timed out waiting for Frontier Nav2 goal cancellation'
            )
            self.get_logger().error(response.message)
            return response

        self._frontier_state = ControlExploration.Request.STATE_IDLE
        response.accepted = True
        response.scheduled = False
        response.state = ControlExploration.Request.STATE_IDLE
        response.message = 'Frontier Explorer stopped and Nav2 goal canceled'
        self.get_logger().info(response.message)
        return response

    async def _wait_for_nav_cancel(self):
        self._waiting_for_nav_cancel = True
        self._stop_wait_future = Future()

        # Nav2 취소 상태가 Frontier STOP 서비스 응답보다 먼저 도착했을
        # 수 있으므로, 대기를 시작하기 전에 캐시된 최신 상태를 확인한다.
        if self._nav_cancel_observed_since_stop_request():
            self._finish_stop_wait(True)
        elif self._stop_timeout_sec <= 0.0:
            self._finish_stop_wait(False)
        else:
            self._stop_timeout_timer = self.create_timer(
                self._stop_timeout_sec,
                self._stop_timeout_callback,
                callback_group=self._callback_group,
            )

        try:
            return await self._stop_wait_future
        finally:
            self._destroy_stop_timeout_timer()
            self._stop_wait_future = None
            self._waiting_for_nav_cancel = False

    def _nav_status_callback(self, msg):
        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        # STOP 대기 여부와 관계없이 항상 최신 Nav2 상태를 캐시한다.
        # 그래야 STOP 서비스 응답 전에 취소가 끝나는 경우도 잡을 수 있다.
        self._nav_goal_active = any(
            status.status in active_states
            for status in msg.status_list
        )
        self._last_nav_status_time = time.monotonic()

        if (
            self._waiting_for_nav_cancel
            and self._nav_cancel_observed_since_stop_request()
        ):
            self._finish_stop_wait(True)

    def _nav_cancel_observed_since_stop_request(self):
        return (
            self._last_nav_status_time >= self._stop_request_time
            and not self._nav_goal_active
        )

    def _stop_timeout_callback(self):
        self._finish_stop_wait(False)

    def _finish_stop_wait(self, stopped):
        future = self._stop_wait_future
        if future is not None and not future.done():
            future.set_result(stopped)

    def _destroy_stop_timeout_timer(self):
        if self._stop_timeout_timer is None:
            return

        self._stop_timeout_timer.cancel()
        self.destroy_timer(self._stop_timeout_timer)
        self._stop_timeout_timer = None

    def _current_response_state(self):
        if self._waiting_for_nav_cancel:
            return ControlExploration.Request.STATE_STOPPING

        if self._frontier_state is not None:
            return self._frontier_state

        return ControlExploration.Request.STATE_IDLE

    def _reject_response(self, response, message, state=None):
        response.accepted = False
        response.scheduled = False
        response.state = (
            self._current_response_state()
            if state is None
            else state
        )
        response.message = message
        return response

    def _copy_response(self, source, destination):
        destination.accepted = source.accepted
        destination.scheduled = source.scheduled
        destination.state = source.state
        destination.message = source.message

    def _log_control_result(self, action, response):
        message = (
            f'Frontier Explorer {action} '
            f'{"accepted" if response.accepted else "rejected"}: '
            f'state={response.state}, message={response.message}'
        )

        if response.accepted:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)


def main(args=None):
    """Run the Frontier state controller node."""
    rclpy.init(args=args)

    node = FrontierStateController()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
