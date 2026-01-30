#!/usr/bin/python3
import blank_package.tape_detect as tape_detect

import os
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from duckietown_msgs.msg import LEDPattern, WheelsCmdStamped
from std_msgs.msg import String, ColorRGBA, Header
from geometry_msgs.msg import Twist
from rclpy.time import Duration


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.output_dir = "/workspace/images/"
        os.makedirs(self.output_dir, exist_ok=True)

        self.vehicle_name = os.getenv('VEHICLE_NAME')

        self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.save_image, 10)

        self.publisher_led = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)
        self.timer_led = self.create_timer(0.1, self.publish_detection)

        self.wheels_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)
        # self.timer_rotate  = self.create_timer(0.1, self.rotate)

        self.spotted_parking = False
        self.obj_x, self.obj_y = 0, 0

        self.frame = None

        self.counter = 0

        self.going_to_parking = False
        self.reached_parking = False

        self.end_amount_of_frames = 0
        self.flash = False

        self.previous_move = None
        self.previous_status = None

    def save_image(self, msg):
        if not self.reached_parking:
            if self.counter % 15 != 0:
                self.counter += 1
                return

            # convert to 1D numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            # convert into BGR
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.frame = frame

            self.spotted_parking, self.obj_x, self.obj_y = tape_detect.detect_parking(frame, self.output_dir,
                                                                                      self.counter)
            self.counter += 1

            self.get_clock().sleep_for(Duration(seconds=0.5))

            # test part
            if self.end_amount_of_frames > 5 and self.going_to_parking:
                self.reached_parking = True
                self.stop_movement()
                self.get_logger().info("Reached parking spot!")
                return

            if self.frame is None:
                return

            if self.previous_status and not self.spotted_parking:
                self.get_logger().info("Lost sight of parking spot, searching...")
                if self.previous_move == 'right':
                    self.turn_left(0.7, 0.1)
                    self.previous_move = 'left'
                elif self.previous_move == 'left':
                    self.turn_right(0.7, 0.1)
                    self.previous_move = 'right'
                self.previous_status = self.spotted_parking



            if not self.spotted_parking:
                self.turn_left(1.0,0.5)
                if self.going_to_parking:
                    self.end_amount_of_frames += 1
            elif self.spotted_parking:
                if not self.going_to_parking:
                    self.going_to_parking = True
                else:
                    self.end_amount_of_frames = 0

                frame_center_x = self.frame.shape[1] // 2
                frame_center_y = self.frame.shape[0] // 2

                error = self.obj_x - frame_center_x
                rotation_speed = 0



                dead_zone = 50  # pixels
                if abs(error) < dead_zone:
                    self.move_forward()
                    self.previous_move = 'forward'
                elif error > 0:
                    self.turn_right(0.9,0.1)
                    self.previous_move = 'right'
                elif error < 0:
                    self.turn_left(0.9,0.1)
                    self.previous_move = 'left'
        self.previous_status = self.spotted_parking


    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheels_pub.publish(wheel_msg)

    def turn_left(self, start_speed,speed):
        self.get_logger().info("Turning left")
        self.run_wheels('right_callback', 0.0, start_speed)
        self.get_clock().sleep_for(Duration(seconds=0.5))
        self.run_wheels('right_callback', 0.0, speed)
        self.get_clock().sleep_for(Duration(seconds=0.5))
        self.run_wheels('stop_callback', 0.0, 0.0)

    def turn_right(self, start_speed,speed):
        self.get_logger().info("Turning left")
        self.run_wheels('right_callback', start_speed, 0.0)
        self.get_clock().sleep_for(Duration(seconds=0.5))
        self.run_wheels('right_callback', speed, 0.0)
        self.get_clock().sleep_for(Duration(seconds=0.5))
        self.run_wheels('stop_callback', 0.0, 0.0)

    def move_forward(self):
        self.get_logger().info("Moving forward")
        self.run_wheels('forward_callback', 0.4, 0.4)
        self.get_clock().sleep_for(Duration(seconds=1))
        self.run_wheels('stop_callback', 0.0, 0.0)

    def stop_movement(self):
        self.get_logger().info("Stopping movement")
        self.run_wheels('stop_callback', 0.0, 0.0)

    def publish_detection(self):
        # LEDPattern is a custom Duckietown Message
        msg = LEDPattern()


        if self.reached_parking:
           ''' self.get_logger().info("Reached parking/trying to switch lights")
            if self.flash == False:
                self.get_logger().info("Trying to switch lights to white")
                pattern = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                self.get_clock().sleep_for(Duration(seconds=0.1))
                flash = True
                return
            if self.flash == True:
                self.get_logger().info("Trying to switch lights to blue")
                pattern = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
                self.get_clock().sleep_for(Duration(seconds=0.1))
                flash = False
                return
'''
           pattern = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        elif self.spotted_parking:
            pattern = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        else:
            pattern = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        # 5 Leds to fill
        msg.rgb_vals = [pattern] * 5

        self.publisher_led.publish(msg)


def main():
    rclpy.init()
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
