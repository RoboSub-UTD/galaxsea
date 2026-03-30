import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import json
import os
import hashlib


class ObstacleTypeMapNode(Node):

    def __init__(self):
        super().__init__('obstacle_type_map_node')

        self.declare_parameter('save_path', '/tmp/obstacle_type_map.json')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('dedup_radius', 0.3)
        self.declare_parameter('marker_scale', 0.2)
        self.declare_parameter('text_scale', 0.15)

        self.save_path    = self.get_parameter('save_path').get_parameter_value().string_value
        self.dedup_radius = self.get_parameter('dedup_radius').get_parameter_value().double_value
        self.marker_scale = self.get_parameter('marker_scale').get_parameter_value().double_value
        self.text_scale   = self.get_parameter('text_scale').get_parameter_value().double_value

        self.obstacles: dict[str, dict] = {}
        self.previous_ids: set[int] = set()

        self.obstacles = {}
        self.save_to_disk()
        self.get_logger().info('Obstacle type map reset')

        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_type_markers', 10)

        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.create_timer(1.0 / rate, self.publish_markers)

        self.get_logger().info('ObstacleTypeMapNode started')

    def add_obstacle(self, x: float, y: float, label: str, radius: float = None) -> str:
        display_radius = radius if radius is not None else self.marker_scale

        for key, obs in self.obstacles.items():
            if obs['label'] == label:
                dx = obs['x'] - x
                dy = obs['y'] - y
                if (dx ** 2 + dy ** 2) ** 0.5 <= self.dedup_radius:
                    obs['x'] = (obs['x'] * obs['count'] + x) / (obs['count'] + 1)
                    obs['y'] = (obs['y'] * obs['count'] + y) / (obs['count'] + 1)
                    obs['radius'] = display_radius
                    obs['count'] += 1
                    self.save_to_disk()
                    return key

        key = f"{label}_{round(x, 2)}_{round(y, 2)}"
        self.obstacles[key] = {
            'x': x,
            'y': y,
            'label': label,
            'radius': display_radius,
            'count': 1
        }
        self.save_to_disk()
        self.get_logger().info(
            f"Added obstacle type '{label}' at ({x:.2f}, {y:.2f}) radius={display_radius:.2f}")
        return key

    def remove_obstacle(self, key: str) -> bool:
        if key in self.obstacles:
            del self.obstacles[key]
            self.save_to_disk()
            return True
        return False

    def lookup(self, x: float, y: float, radius: float = 0.5) -> list[dict]:
        results = []
        for obs in self.obstacles.values():
            dx = obs['x'] - x
            dy = obs['y'] - y
            dist = (dx ** 2 + dy ** 2) ** 0.5
            if dist <= radius:
                results.append({**obs, 'distance': dist})
        results.sort(key=lambda o: o['distance'])
        return results

    def get_all(self) -> dict:
        return dict(self.obstacles)

    def publish_markers(self):
        array = MarkerArray()
        active_ids = set()

        for i, (key, obs) in enumerate(self.obstacles.items()):
            stamp = self.get_clock().now().to_msg()
            color = self._color_for_label(obs['label'])
            active_ids.add(i)

            circle = Marker()
            circle.header.frame_id = 'map'
            circle.header.stamp = stamp
            circle.ns = 'obstacle_type_circles'
            circle.id = i
            circle.type = Marker.CYLINDER
            circle.action = Marker.ADD
            circle.pose.position.x = obs['x']
            circle.pose.position.y = obs['y']
            circle.pose.position.z = 0.0
            circle.pose.orientation.w = 1.0
            circle.scale.x = obs['radius']
            circle.scale.y = obs['radius']
            circle.scale.z = 0.01
            circle.color = color
            array.markers.append(circle)

            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = stamp
            text.ns = 'obstacle_type_text'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = obs['x']
            text.pose.position.y = obs['y']
            text.pose.position.z = 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = self.text_scale
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = obs['label']
            array.markers.append(text)

        for old_id in self.previous_ids - active_ids:
            for ns in ('obstacle_type_circles', 'obstacle_type_text'):
                delete = Marker()
                delete.header.frame_id = 'map'
                delete.ns = ns
                delete.id = old_id
                delete.action = Marker.DELETE
                array.markers.append(delete)

        self.previous_ids = active_ids
        self.marker_pub.publish(array)

    def save_to_disk(self):
        try:
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            with open(self.save_path, 'w') as f:
                json.dump(self.obstacles, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f'Failed to save obstacle type map: {e}')

    def load_from_disk(self):
        if os.path.exists(self.save_path):
            try:
                with open(self.save_path) as f:
                    self.obstacles = json.load(f)
            except Exception as e:
                self.get_logger().warn(f'Failed to load obstacle type map: {e}')
                self.obstacles = {}

    def _color_for_label(self, label: str) -> ColorRGBA:
        h = int(hashlib.md5(label.encode()).hexdigest(), 16)
        r = ((h >> 16) & 0xFF) / 255.0
        g = ((h >> 8) & 0xFF) / 255.0
        b = (h & 0xFF) / 255.0
        return ColorRGBA(r=r, g=g, b=b, a=0.85)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTypeMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()