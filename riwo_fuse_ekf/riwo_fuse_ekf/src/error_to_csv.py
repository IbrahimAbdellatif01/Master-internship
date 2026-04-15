#!/usr/bin/env python3
import csv
import os
from datetime import datetime
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray

class ErrorToCsv(Node):
    def __init__(self):
        super().__init__('error_to_csv')

        # declare parameters
        self.declare_parameter('input_topic', 'localization_error_fuse')
        self.declare_parameter('output_file', '')

        #get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # generate output filename
        if not output_file:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_file = f'localization_error_{timestamp}.csv'

        # csv setup
        self.csv_file = open(output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # write the header
        self.csv_writer.writerow([
            'timestamp_sec',
            'timestamp_nanosec',
            # ground truth
            'truth_x',
            'truth_y',
            'truth_z',
            'truth_roll',
            'truth_pitch',
            'truth_yaw',
            # sensor readings
            'loc_x',
            'loc_y',
            'loc_z',
            'loc_roll',
            'loc_pitch',
            'loc_yaw',
            # error
            'error_x',
            'error_y',
            'error_z',
            'error_roll',
            'error_pitch',
            'error_yaw',
            'time_difference'
        ])    

        # create subscriber
        self.subscription = self.create_subscription(
            DiagnosticArray,
            input_topic,
            self.callback,
            10
        )

        self.get_logger().info(
            f'logging "{input_topic}" to "{output_file}"'
        )

        def callback(self,msg: DiagnosticArray):
            # extract timestamp

            stamp_sec = msg.header.stamp.sec
            stamp_nanosec = msg.header.stamp.nanosec
            # parse the data into a dictonary

            data = {}
            for status in msg.status:
                for kv in status.values:
                    data[kv.key]= kv.value

            try:
                self.csv_writer.writerow([
                    stamp_sec,
                    stamp_nanosec,
                    data.get('truth x',''),
                    data.get('truth y',''),
                    data.get('truth z',''),
                    data.get('truth roll',''),
                    data.get('truth pitch',''),
                    data.get('data yaw',''),
                    data.get('localization x',''),
                    data.get('localization y',''),
                    data.get('localization z',''),
                    data.get('localization roll',''),
                    data.get('localization pitch',''),
                    data.get('localization yaw',''),
                    data.get('error x',''),
                    data.get('error y',''),
                    data.get('error z',''),
                    data.get('error roll',''),
                    data.get('error pitch',''),
                    data.get('error yaw',''),
                    data.get('time difference',''),
                ])
                self.csv_file.flush() # flush it after each rowthe data can not be lost
            
            except Exception as e:
                self.get_logger().error(f'failed to write csv row: {e}')
                super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ErrorToCsv()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__== '__main__':
    main()