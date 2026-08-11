from fire_vla_core.adapters.mock_adapters import MockNavigationAdapter, MockReportAdapter, MockResultQueue, MockSprayAdapter, MockWaitAdapter
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import Action, ActionSubmissionStatus, ActionType, Pose2D


def make_dispatcher():
    queue = MockResultQueue()
    return ActionDispatcher(MockNavigationAdapter(queue), MockSprayAdapter(queue), MockReportAdapter(queue), MockWaitAdapter(queue)), queue


def test_all_ports_use_submit_contract():
    dispatcher, queue = make_dispatcher()
    submission = dispatcher.submit(Action("a1", ActionType.NAVIGATE_TO, "test", target="x", target_pose=Pose2D(1, 2)))
    assert submission.status == ActionSubmissionStatus.ACCEPTED
    assert len(queue.drain_results()) == 1


def test_duplicate_action_id_is_not_submitted_twice():
    dispatcher, queue = make_dispatcher()
    action = Action("a1", ActionType.WAIT, "test")
    assert dispatcher.submit(action).status == ActionSubmissionStatus.ACCEPTED
    assert dispatcher.submit(action).status == ActionSubmissionStatus.DUPLICATE
    assert len(queue.drain_results()) == 1
