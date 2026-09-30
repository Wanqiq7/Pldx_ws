from unittest.mock import Mock

from geometry_msgs.msg import Twist

from sim_nav_adapter.cmd_vel_adapter import SimulationCmdVelAdapter
from sim_nav_adapter.velocity_command import PlanarCommand


def test_behavior_callback_does_not_publish_an_ignored_command():
    adapter = Mock()
    adapter._command_from_twist.return_value = PlanarCommand.zero()
    adapter._odom_age.return_value = 0.0
    adapter.gate.accept_behavior.return_value = None

    SimulationCmdVelAdapter._behavior_callback(adapter, Twist())

    adapter._publish.assert_not_called()


def test_behavior_callback_publishes_the_selected_command():
    adapter = Mock()
    command = PlanarCommand(-0.1, 0.0, 0.5)
    adapter._command_from_twist.return_value = command
    adapter._odom_age.return_value = 0.0
    adapter.gate.accept_behavior.return_value = command

    SimulationCmdVelAdapter._behavior_callback(adapter, Twist())

    adapter._publish.assert_called_once_with(command)
