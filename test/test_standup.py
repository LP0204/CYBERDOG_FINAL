"""离线检查起身失败后，定时器和退出路径都不会下发伺服指令。"""

from concurrent.futures import Future
from importlib import util
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


def load_adapter():
    # 仅替代模块导入所需的 ROS 类型，测试运行生产节点的 tick 和 shutdown。
    imports = {
        'rclpy': SimpleNamespace(),
        'rclpy.node': SimpleNamespace(Node=object),
        'rclpy.qos': SimpleNamespace(qos_profile_sensor_data=object()),
        'geometry_msgs.msg': SimpleNamespace(PoseStamped=object),
        'std_msgs.msg': SimpleNamespace(String=object),
        'protocol.msg': SimpleNamespace(MotionServoCmd=object),
        'protocol.srv': SimpleNamespace(MotionResultCmd=SimpleNamespace(Request=SimpleNamespace)),
    }
    spec = util.find_spec('soccer.ros_node')
    module = util.module_from_spec(spec)
    with patch.dict(sys.modules, imports):
        spec.loader.exec_module(module)
    return module


adapter = load_adapter()


class StandupTest(unittest.TestCase):
    def setUp(self):
        self.node = adapter.SoccerNode.__new__(adapter.SoccerNode)
        self.node.phase = 'standup'
        self.node.dry_run = False
        self.node.stand_t0 = 0.0
        self.node.stand_client = self.node.stand_future = None
        self.node.dog = SimpleNamespace(namespace='test_dog')
        self.node.get_logger = Mock()
        self.node.create_client = Mock()
        self.node.send_halt = Mock()
        self.node.pose_of = Mock()
        self.client = self.node.create_client.return_value
        self.client.service_is_ready.return_value = True
        self.future = Future()
        self.client.call_async.return_value = self.future

    def tick(self, t):
        with patch.object(adapter.time, 'monotonic', return_value=t):
            self.node.tick()

    def assert_locked(self):
        self.assertEqual(self.node.phase, 'stand_failed')
        request_count = self.client.call_async.call_count
        self.tick(30.0)
        self.node.shutdown()
        self.assertEqual(self.client.call_async.call_count, request_count)
        self.node.pose_of.assert_not_called()
        self.node.send_halt.assert_not_called()
        self.node.get_logger.return_value.error.assert_called_once()

    def test_only_successful_response_starts_play(self):
        self.tick(1.0)
        self.tick(1.1)
        self.assertEqual(self.node.phase, 'standup')
        self.client.call_async.assert_called_once()
        self.assertEqual(self.client.call_async.call_args[0][0].motion_id, 111)
        self.assertEqual(self.client.call_async.call_args[0][0].cmd_source, 4)
        self.future.set_result(SimpleNamespace(result=True, code=0))
        self.tick(1.2)
        self.assertEqual(self.node.phase, 'run')

    def test_rejected_response_prevents_play(self):
        self.tick(1.0)
        self.future.set_result(SimpleNamespace(result=False, code=3052))
        self.tick(1.1)
        self.assert_locked()

    def test_nonzero_code_prevents_play(self):
        self.tick(1.0)
        self.future.set_result(SimpleNamespace(result=True, code=3052))
        self.tick(1.1)
        self.assert_locked()

    def test_response_exception_prevents_play(self):
        self.tick(1.0)
        self.future.set_exception(RuntimeError('transport error'))
        self.tick(1.1)
        self.assert_locked()

    def test_service_discovery_timeout(self):
        self.client.service_is_ready.return_value = False
        self.tick(1.0)
        self.tick(5.0)
        self.assert_locked()
        self.client.call_async.assert_not_called()

    def test_late_success_cannot_restart_play(self):
        self.tick(1.0)
        self.tick(16.0)
        self.future.set_result(SimpleNamespace(result=True, code=0))
        self.assert_locked()

    def test_missing_service_type_prevents_play(self):
        with patch.object(adapter, 'MotionResultCmd', None):
            self.tick(1.0)
        self.assert_locked()
        self.node.create_client.assert_not_called()

    def test_preview_runs_without_stand_service(self):
        self.node.dry_run = True
        with patch.object(adapter, 'MotionResultCmd', None):
            self.tick(1.0)
        self.assertEqual(self.node.phase, 'run')
        self.node.create_client.assert_not_called()
