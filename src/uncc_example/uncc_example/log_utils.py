from rclpy.logging import LoggingSeverity


def silence_default_logger(node):
    """기본 로거를 ERROR만 출력하도록 낮춘다."""
    node.get_logger().set_level(LoggingSeverity.ERROR)


def make_event_logger(node):
    """기본 로거는 ERROR만 출력하도록 낮추고, state 전환/객체 인식/
    nav2 goal/화재진압 판정처럼 동작과 직접 관련된 로그만 남기기 위한
    INFO 레벨 child 로거를 반환한다."""
    silence_default_logger(node)
    event_logger = node.get_logger().get_child('event')
    event_logger.set_level(LoggingSeverity.INFO)
    return event_logger
