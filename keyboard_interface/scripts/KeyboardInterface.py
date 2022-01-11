#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pynput.keyboard import Listener

from keyboard_interface.msg import Keys

class KeyboardInterface(Node):

  def __init__(self):
    super().__init__('keyboard_interface')
    self.publisher_ = self.create_publisher(Keys, '/keyboard', 30)
    timer_period = 0.1
    self.keys_msg = Keys()
    self.timer = self.create_timer(timer_period, self.timer_callback)
    listener = Listener(on_press=self.onPress, on_release=self.onRelease)
    listener.start()
  
  def timer_callback(self):
    self.publisher_.publish(self.keys_msg)

  def onRelease(self, key):
    key = str(key).upper()
    if key in self.keys_msg.pressed_keys:
      self.keys_msg.pressed_keys.remove(key)
      self.publisher_.publish(self.keys_msg)

  def onPress(self, key):
    key = str(key).upper()
    if key not in self.keys_msg.pressed_keys:
      self.keys_msg.pressed_keys.append(key)
      print(key)
      self.publisher_.publish(self.keys_msg)




def main(args=None):
  rclpy.init(args=args)

  keyboard_interface = KeyboardInterface()

  rclpy.spin(keyboard_interface)

  keyboard_interface.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

