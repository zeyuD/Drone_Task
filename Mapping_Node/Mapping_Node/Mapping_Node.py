
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from .occupancy_grid import OccupancyGridMap


class MappingNode(Node):
    """
    Mapping Node that subscribes to obstacle inputs and publishes /map.
    Obstacle updates are received from /obstacles.
    """

    def __init__(self):
        super().__init__('Mapping_Node')
        self.grid = OccupancyGridMap(
            width_meters=8,
            height_meters=8,
            resolution=0.2,
            inflation_meters=0.0
        )
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.obstacles_sub = self.create_subscription(PoseArray, 'object_detections', self.obstacles_callback, 10)
        self.publish_timer = self.create_timer(0.2, self.publish_map)
        self._has_obstacles = False

        self.get_logger().info(
            'Mapping Node initialized: subscribing to /object_detections, publishing /map'
        )

    def obstacles_callback(self, msg):
        """
        Receives obstacle points from /object_detections as PoseArray.
        Mapping convention: x/y from pose.position, optional radius in pose.position.z.
        """
        obstacles = []
        for pose in msg.poses:
            radius_m = pose.position.z if pose.position.z > 0.0 else 0.0
            obstacles.append(
                {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'radius_m': radius_m,
                }
            )

        if obstacles:
            self.grid.update_from_obstacles(obstacles)
            self._has_obstacles = True

    def publish_map(self):
        """Publishes the current occupancy grid on /map."""

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.header.frame_id = 'map'

        msg.info.resolution = float(self.grid.resolution)
        msg.info.width = int(self.grid.grid_width)
        msg.info.height = int(self.grid.grid_height)
        msg.info.origin.position.x = -self.grid.width_meters / 2.0
        msg.info.origin.position.y = -self.grid.height_meters / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        data = []
        for row in self.grid.grid:
            for cell in row:
                if cell == OccupancyGridMap.UNKNOWN:
                    data.append(-1)
                elif cell == OccupancyGridMap.OCCUPIED:
                    data.append(100)
                else:
                    data.append(0)

        msg.data = data
        self.map_pub.publish(msg)

        if not self._has_obstacles:
            self.get_logger().debug('Published /map with no obstacle updates yet')


def main(args=None):
    rclpy.init(args=args)

    node = MappingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
