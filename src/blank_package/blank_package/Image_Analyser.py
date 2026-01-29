#!/usr/bin/python3
import blank_package.tape_detect as tape_detect

import os
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.output_dir = "/workspace/images/"
        os.makedirs(self.output_dir, exist_ok=True)

        self.vehicle_name = os.getenv('VEHICLE_NAME')

        self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.save_image, 10)

        self.publisher_led = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)
        self.timer_led = self.create_timer(1, self.publish_detection)

        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.vehicle_name}/cmd_vel', 1)
        self.timer_rotate  = self.create_timer(0.1, self.rotate)

        self.spotted_parking = False
        self.obj_x, self.obj_y = 0, 0

        self.frame = None

        self.counter = 0

    def save_image(self, msg):
        if self.counter % 30 != 0:
            self.counter += 1
            return
        with open(self.output_dir + str(self.counter) + '.jpg', 'wb') as f:
            self.get_logger().info(f'Saving image {self.counter}')
            f.write(msg.data)

        #convert to 1D numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        #convert into BGR
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.frame = frame

        self.spotted_parking, self.obj_x, self.obj_y = tape_detect.detect_parking(frame)
        self.counter += 1

    def rotate(self):
        if self.frame is None:
            return 
        
        frame_center_x = self.frame.shape[1] // 2
        frame_center_y = self.frame.shape[0] // 2

        error = self.obj_x - frame_center_x
        rotation_speed = 0

        dead_zone = 10  # pixels
        if abs(error) < dead_zone:
            rotation_speed = 0 
        elif error > 0:
            rotation_speed = 0.3
        if error < 0:
            rotation_speed = -0.3
        
        cmd = Twist()
        cmd.angular.z = rotation_speed 
        self.cmd_vel_pub.publish(cmd) 



    def publish_detection(self):
        # LEDPattern is a custom Duckietown Message
        msg = LEDPattern()

        if self.spotted_parking:
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