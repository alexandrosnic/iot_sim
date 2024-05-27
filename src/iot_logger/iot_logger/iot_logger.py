import json
import os
import time

import rclpy
from rclpy.node import Node
import rospkg
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from sensor_msgs.msg import Image


class DataLogger(Node):

    def __init__(self):
        super().__init__('iot_logger')
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.logger,
            10)

        # Local logging file
        base_path = os.path.dirname(os.path.realpath(__file__))
        self.logger_file = os.path.abspath(os.path.join(base_path, '../../../../../../data/logger.json'))
        self.offline_logger = self.load_json(self.logger_file)

    def logger(self, msg):

        # Get image data
        timestamp = time.time()
        frame_id = msg.header.frame_id
        height = msg.height
        width = msg.width
        encoding = msg.encoding
        pixel_data = msg.data

        pixel_values = list(pixel_data)[:100]

        image_data = {
            'timestamp': timestamp,
            'frame_id': frame_id,
            'height': height,
            'width': width,
            'encoding': encoding,
            'pixel_values': pixel_values
        }

        # Write image data
        json_data = json.dumps(image_data)
        self.offline_logger.append(json_data)
        self.write_json(self.logger_file)

    def write_json(self, file):
        try:
            with open(file, 'w') as f:
                json.dump(self.offline_logger, f, indent=4)
            # self.get_logger().info(f'Data successfully written to {self.logger_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to write data to file: {e}')

    def load_json(self, file):
        try:
            if os.path.exists(file):
                with open(file, 'r') as f:
                    data = f.read()
                    if data:  
                        return json.loads(data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to load JSON data from file: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to read file: {e}')
        return []

def main(args=None):
    rclpy.init(args=args)
    iot_logger = DataLogger()
    rclpy.spin(iot_logger)
    iot_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()