"""
    Script that listens to a PointCloud2 topic and saves the points to a CSV file.

    To run it use python3 pointcloud2_to_csv.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


TOPIC: str = 'slp_14h_001/front_laser_pile_sensor/points'
FILE_OUT = 'pointcloud_03.csv'


class PointCloud2ToCSV(Node):

    def __init__(self):
        super().__init__('pointcloud2_to_csv')
        self.subscription = self.create_subscription(
            PointCloud2,
            TOPIC,
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('Received PointCloud2 message')
        # Convert PointCloud2 message to a list of points
        point_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            point_list.append(point)

       # Export pointlist_into a text file with x, y, z withot using pands
        with open(FILE_OUT, 'w') as f:
            for point in point_list:
                f.write(f'{point[0]},{point[1]},{point[2]}\n')

        # Shutdown the node
        self.get_logger().info('Shutting down node')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    pointcloud2_to_csv = PointCloud2ToCSV()

    try:
        rclpy.spin(pointcloud2_to_csv)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        pointcloud2_to_csv.get_logger().error('Error in pointcloud2_to_csv node: ' + str(e))

if __name__ == '__main__':
    main()