#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from keyboard_interface.msg import Keys
from keyboard_interface.msg import KeyEvent

try:
  from pynput.keyboard import Listener
except ImportError as e:
  import pip
  print("pynput package is required. Installing...")
  pip.main(['install', 'pynput'])
  from pynput.keyboard import Listener


class KeyboardInterface(Node):

  def __init__(self):
    super().__init__('keyboard_interface')
    self.keys_publisher_ = self.create_publisher(Keys, '/keys', 10)
    self.key_event_publisher_ = self.create_publisher(KeyEvent, '/key_event', 10)

    self.sticky_buttons = self.declare_parameter('sticky_buttons', False).get_parameter_value().bool_value
    self.keys_publish_rate = self.declare_parameter('keys_publish_rate', 10.0).get_parameter_value().double_value

    # self.sticky_buttons = self.get_parameter('sticky_buttons').get_parameter_value().bool_value
    # self.keys_publish_rate = self.get_parameter('keys_publish_rate').get_parameter_value().double_value

    self.keys_msg = Keys()
    self.timer = self.create_timer(1/self.keys_publish_rate, self.timerCallback)
    listener = Listener(on_press=self.onPress, on_release=self.onRelease)
    listener.start()
  
  def timerCallback(self):
    self.keys_publisher_.publish(self.keys_msg)

  def onRelease(self, key):
    key = str(key).upper()
    key_event_msg = KeyEvent()
    key_event_msg.event = KeyEvent.RELEASED
    key_event_msg.key = key
    self.key_event_publisher_.publish(key_event_msg)

    if not self.sticky_buttons:
      if key in self.keys_msg.pressed_keys:
        self.keys_msg.pressed_keys.remove(key)


  def onPress(self, key):
    key = str(key).upper()
    key_event_msg = KeyEvent()
    key_event_msg.event = KeyEvent.PRESSED
    key_event_msg.key = key
    self.key_event_publisher_.publish(key_event_msg)

    if key not in self.keys_msg.pressed_keys:
      self.keys_msg.pressed_keys.append(key)
    elif self.sticky_buttons:
      self.keys_msg.pressed_keys.remove(key)


def main(args=None):
  rclpy.init(args=args)

  keyboard_interface = KeyboardInterface()

  rclpy.spin(keyboard_interface)

  keyboard_interface.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

