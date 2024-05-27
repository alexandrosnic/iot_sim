import rclpy
from rclpy.node import Node

import random
import os
import json
import time
import requests
import threading  

class DataUploader(Node):
    def __init__(self):
        super().__init__('iot_uploader')

        base_path = os.path.dirname(os.path.realpath(__file__))
        self.logger_file = os.path.abspath(os.path.join(base_path, '../../../../../../data/logger.json'))
        self.logger_data = self.load_json(self.logger_file)

        self.cloud_target = 'https://upload-data-from-iot.com/upload'
        self.connected = False

        self.timer = threading.Timer(30.0, self.check_connectivity)
        self.timer.start()

    def load_json(self, file):
        try:
            if os.path.exists(file):
                with open(file, 'r') as f:
                    data = json.load(f)
                    return data
        except Exception as e:
            self.get_logger().error(f'Failed to load data from file: {e}')
        return []

    def transfer_data(self):
        if self.connected:
            try:
                # Send data to cloud target
                response = requests.post(self.cloud_service_endpoint, json=self.data)
                if response.status_code == 200:
                    self.get_logger().info('Data transfer to cloud successful.')
                    # Clear the data file after successful transfer
                    open(self.logger_file, 'w').close()
                else:
                    self.get_logger().error(f'Failed to transfer data to cloud: {response.status_code}')
            except Exception as e:
                self.get_logger().error(f'Failed to transfer data to cloud: {e}')
        else:
            self.get_logger().warning('No network connectivity. Data transfer postponed.')

    def check_connectivity(self):
        # Check network connection and upload if connected
        self.connected = True 
        if self.connected:
            self.transfer_data()
        else:
            self.get_logger().warning('No network connectivity. Data transfer postponed.')
        # Restart timer
        self.timer = threading.Timer(30.0, self.check_connectivity)
        self.timer.start()

    def stop_timer(self):
        if self.timer.is_alive():
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    iot_uploader = DataUploader()
    try:
        rclpy.spin(iot_uploader)
    finally:
        # Make sure that the thread is destroyed
        iot_uploader.stop_timer()
        iot_uploader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
