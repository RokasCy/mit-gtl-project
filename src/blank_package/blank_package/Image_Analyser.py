#!/usr/bin/python3

import tape_detect

import os
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.output_dir = "/workspace/images/"
        os.makedirs(self.output_dir, exist_ok=True)
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        self.counter = 0
        self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.save_image, 10)
        self.publisher = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)
        self.timer = self.create_timer(1, self.publish_pattern)

        self.spotted_parking = False

    def save_image(self, msg):
        if self.counter % 30 != 0:
            self.counter += 1
            return
        with open(self.output_dir + str(self.counter) + '.jpg', 'wb') as f:
            self.get_logger().info(f'Saving image {self.counter}')
            f.write(msg.data)

        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        self.spotted_parking = tape_detect.detect_parking(np_arr)

        self.counter += 1

    def pattern_analyser(self):
        # LEDPattern is a custom Duckietown Message
        msg = LEDPattern()

        if self.spotted_parking:
            pattern = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        else:
            pattern = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        # 5 Leds to fill
        msg.rgb_vals = [pattern] * 5

        self.publisher.publish(msg)
        self.counter += 1

def main():
    rclpy.init()
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()