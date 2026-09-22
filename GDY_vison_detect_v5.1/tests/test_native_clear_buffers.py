import unittest
from unittest.mock import Mock, call

from handeye_calib.native_camera.worker import clear_and_trigger


class ClearBuffersTests(unittest.TestCase):
    @staticmethod
    def check(rc):
        if rc != 0:
            raise RuntimeError("SDK failure")

    def test_every_capture_clears_both_before_trigger(self):
        lib = Mock()
        lib.v5_clear_frame_buffer.return_value = 0
        lib.st_trigger.return_value = 0
        for _ in range(2):
            clear_and_trigger(lib, self.check)
        self.assertEqual(lib.mock_calls, [call.v5_clear_frame_buffer(0),
                         call.v5_clear_frame_buffer(1), call.st_trigger()] * 2)

    def test_either_clear_failure_prevents_trigger(self):
        for results in ([-1], [0, -1]):
            with self.subTest(results=results):
                lib = Mock()
                lib.v5_clear_frame_buffer.side_effect = results
                with self.assertRaises(RuntimeError):
                    clear_and_trigger(lib, self.check)
                lib.st_trigger.assert_not_called()
                self.assertEqual(lib.v5_clear_frame_buffer.call_count, len(results))

    def test_trigger_failure_is_not_retried(self):
        lib = Mock()
        lib.v5_clear_frame_buffer.return_value = 0
        lib.st_trigger.return_value = -1
        with self.assertRaises(RuntimeError):
            clear_and_trigger(lib, self.check)
        lib.st_trigger.assert_called_once_with()
