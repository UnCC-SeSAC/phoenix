from types import SimpleNamespace

from uncc_example import state_manager as state_manager_module
from uncc_example.state_manager import StateManager


class Logger:
    def warn(self, message, **kwargs):
        pass


def manager():
    node = StateManager.__new__(StateManager)
    node.latest_battery = None
    node.low_battery_threshold = 7000
    node.low_battery_confirm_sec = 3.0
    node._battery_low_state = False
    node._battery_pending_value = None
    node._battery_pending_since = None
    node.get_logger = lambda: Logger()
    return node


def test_low_battery_requires_continuous_confirmation(monkeypatch):
    node = manager()
    now = [100.0]
    monkeypatch.setattr(state_manager_module.time, 'time', lambda: now[0])

    node.battery_callback(SimpleNamespace(data=6900))
    assert node.is_battery_low() is False

    now[0] = 102.9
    assert node.is_battery_low() is False

    now[0] = 103.0
    assert node.is_battery_low() is True


def test_battery_recovery_is_debounced(monkeypatch):
    node = manager()
    now = [200.0]
    monkeypatch.setattr(state_manager_module.time, 'time', lambda: now[0])

    node._battery_low_state = True
    node._battery_pending_value = True
    node._battery_pending_since = 190.0
    node.battery_callback(SimpleNamespace(data=7100))

    assert node.is_battery_low() is True
    now[0] = 203.0
    assert node.is_battery_low() is False
