import unittest
from unittest.mock import patch
from arm_gui_controller import GuiControllerNode
from rover.msg import ArmInputs
from std_msgs.msg import String, Int32
import rosbag

class TestGuiControllerNode(unittest.TestCase):

    @patch('rospy.Publisher')
    @patch('rospy.init_node')
    def setUp(self, mock_init_node, mock_publisher):
        self.mock_publisher = mock_publisher
        self.node = GuiControllerNode()

    def test_on_press_forward(self):
        with patch.object(self.node.inputPublisher, 'publish') as mock_publish:
            self.node.on_press('Forward')
            mock_publish.assert_called_once()
            published_msg = mock_publish.call_args[0][0]
            self.assertIsInstance(published_msg, ArmInputs)
            self.assertEqual(published_msg.l_vertical, 1)
            self.assertEqual(published_msg.l_horizontal, 0)

    def test_on_press_backward(self):
        with patch.object(self.node.inputPublisher, 'publish') as mock_publish:
            self.node.on_press('Backward')
            mock_publish.assert_called_once()
            published_msg = mock_publish.call_args[0][0]
            self.assertIsInstance(published_msg, ArmInputs)
            self.assertEqual(published_msg.l_vertical, -1)
            self.assertEqual(published_msg.l_horizontal, 0)

    def test_on_press_left(self):
        with patch.object(self.node.inputPublisher, 'publish') as mock_publish:
            self.node.on_press('Left')
            mock_publish.assert_called_once()
            published_msg = mock_publish.call_args[0][0]
            self.assertIsInstance(published_msg, ArmInputs)
            self.assertEqual(published_msg.l_horizontal, 1)
            self.assertEqual(published_msg.l_vertical, 0)

    def test_on_press_right(self):
        with patch.object(self.node.inputPublisher, 'publish') as mock_publish:
            self.node.on_press('Right')
            mock_publish.assert_called_once()
            published_msg = mock_publish.call_args[0][0]
            self.assertIsInstance(published_msg, ArmInputs)
            self.assertEqual(published_msg.l_horizontal, -1)
            self.assertEqual(published_msg.l_vertical, 0)

    def test_on_press_manual(self):
        with patch.object(self.node.statePublisher, 'publish') as mock_publish:
            self.node.on_press('Manual')
            mock_publish.assert_called_once_with('Manual')

    def test_on_release(self):
        with patch.object(self.node.inputPublisher, 'publish') as mock_publish:
            self.node.on_release()
            mock_publish.assert_called_once()
            published_msg = mock_publish.call_args[0][0]
            self.assertIsInstance(published_msg, ArmInputs)
            self.assertEqual(published_msg.l_vertical, 0)
            self.assertEqual(published_msg.l_horizontal, 0)

if __name__ == '__main__':
    unittest.main()