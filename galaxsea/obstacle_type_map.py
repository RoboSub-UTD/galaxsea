import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformListener
import json
import os
import hashlib
import math


class ObstacleTypeMapNode(Node):

    def __init__(self):
        super().__init__('obstacle_type_map_node')

        self.declare_parameter('save_path', '/tmp/obstacle_type_map.json')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('dedup_radius', 0.15)
        self.declare_parameter('marker_scale', 0.2)
        self.declare_parameter('text_scale', 0.15)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.save_path = self.get_parameter('save_path').get_parameter_value().string_value
        self.dedup_radius = self.get_parameter('dedup_radius').get_parameter_value().double_value
        self.marker_scale = self.get_parameter('marker_scale').get_parameter_value().double_value
        self.text_scale = self.get_parameter('text_scale').get_parameter_value().double_value

        self.obstacles: dict[str, dict] = {}
        self.previous_ids: set[int] = set()
        self.save_to_disk()
        self.get_logger().info('Obstacle type map reset')

        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_type_markers', 10)
        self.res_pub = self.create_publisher(String, '/obstacle_map/response', 10)
        self.req_sub = self.create_subscription(String, '/obstacle_map/request', self.handle_request, 10)

        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.create_timer(1.0 / rate, self.publish_markers)
        self.get_logger().info('ObstacleTypeMapNode started')

    def handle_request(self, msg: String):
        try:
            req = json.loads(msg.data)
            action = req.get("action")
            if action == "add_obstacle":
                res = self.add_obstacle_request(req["x"], req["y"], req["label"], req.get("radius"))
            elif action == "remove_obstacle":
                res = self.remove_obstacle_request(req["key"])
            elif action == "lookup":
                res = self.lookup_request(req["x"], req["y"])
            elif action == "get_closest":
                res = self.get_closest_request(req.get("n", 1), req.get("label"), req.get("exclude_used", True))
            elif action == "mark_used":
                res = self.mark_used_request(req["key"])
            elif action == "get_all":
                res = self.get_all_request()
            elif action == "sum_distances_in_radius":
                res = self.sum_obstacle_distance_request(req["x"], req["y"], req["radius"], req.get("label"))
            elif action == "min_cost_on_line":
                res = self.min_cost_on_line_request(req["x1"], req["y1"], req["x2"], req["y2"], req["label"], req["size"])
            else:
                res = {"error": f"Unknown action: {action}"}
        except Exception as e:
            res = {"error": str(e)}
        if req.get("req_id"):
            res["req_id"] = req.get("req_id")
        self.res_pub.publish(String(data=json.dumps(res)))

    def add_obstacle_request(self, x: float, y: float, label: str, radius: float = None) -> dict:
        key = self.add_obstacle(x, y, label, radius)
        return {"key": key}

    def remove_obstacle_request(self, key: str) -> dict:
        success = self.remove_obstacle(key)
        return {"success": success}

    def lookup_request(self, x: float, y: float) -> dict:
        results = self.lookup(x, y)
        return {
            "keys": [k for k, _ in results],
            "distances": [r['distance'] for _, r in results],
            "labels": [r['label'] for _, r in results]
        }

    def get_closest_request(self, n: int = 1, label: str = None, exclude_used: bool = True) -> dict:
        results = self.get_closest(n, label, exclude_used)
        return {
            "keys": [r[0] for r in results],
            "xs": [r[1]['x'] for r in results],
            "ys": [r[1]['y'] for r in results],
            "labels": [r[1]['label'] for r in results]
        }
    
    def sum_obstacle_distance_request(self, x: float, y: float, radius: float, label: str = None) -> dict:
        total = self.sum_obstacle_distance(x, y, radius, label)
        return {"total_distance": total}

    def mark_used_request(self, key: str) -> dict:
        self.mark_used(key)
        return {"success": key in self.obstacles}

    def get_all_request(self) -> dict:
        return {"json_data": json.dumps(self.get_all())}
    
    def min_cost_on_line_request(self, x1: float, y1: float, x2: float, y2: float, label: str = None, size: float = 10) -> dict:
        t = self.min_cost_on_line(x1, x2, y1, y2, label, size)
        return {"scalar_value": t}

    def add_obstacle(self, x: float, y: float, label: str, radius: float = None) -> str:
        display_radius = radius if radius is not None else self.marker_scale
        for key, obs in self.obstacles.items():
            if obs['label'] == label:
                dx = obs['x'] - x
                dy = obs['y'] - y
                if (dx * dx + dy * dy) ** 0.5 <= max(self.dedup_radius, obs['radius']):
                    obs['x'] = (obs['x'] * obs['count'] + x) / (obs['count'] + 1)
                    obs['y'] = (obs['y'] * obs['count'] + y) / (obs['count'] + 1)
                    obs['radius'] = display_radius
                    obs['count'] += 1
                    self.save_to_disk()
                    return key
        key = f"{label}_{round(x, 2)}_{round(y, 2)}"
        self.obstacles[key] = {'x': x, 'y': y, 'label': label, 'radius': display_radius, 'count': 1, 'used': False}
        self.save_to_disk()
        self.get_logger().info(f"Added obstacle '{label}' at ({x:.2f}, {y:.2f}) radius={display_radius:.2f}")
        return key

    def remove_obstacle(self, key: str) -> bool:
        if key in self.obstacles:
            del self.obstacles[key]
            self.save_to_disk()
            return True
        return False

    def lookup(self, x: float, y: float) -> list[tuple[str, dict]]:
        results = []
        for key, obs in self.obstacles.items():
            dx = obs['x'] - x
            dy = obs['y'] - y
            dist2 = dx * dx + dy * dy
            if dist2 <= obs['radius'] * obs['radius']:
                results.append((key, {**obs, 'distance': dist2 ** 0.5}))
        results.sort(key=lambda o: o[1]['distance'])
        return results

    def get_boat_position(self) -> tuple[float, float]:
        try:
            t = self.tf_buffer.lookup_transform('map', 'wamv/wamv/base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed, defaulting to origin: {e}')
            return 0.0, 0.0

    def get_closest(self, n: int = 1, label: str = None, exclude_used: bool = True) -> list[tuple[str, dict]]:
        rx, ry = self.get_boat_position()
        matches = [(key, obs) for key, obs in self.obstacles.items()
                   if (not exclude_used or not obs.get('used', False))
                   and (label is None or obs['label'] == label)]
        matches.sort(key=lambda o: (o[1]['x'] - rx) ** 2 + (o[1]['y'] - ry) ** 2)
        return matches[:n]
    
    def sum_obstacle_distance(self, x: float, y: float, radius: float, label: str = None) -> float:
        total = 0.0
        for key, obs in self.obstacles.items():
            dx = obs['x'] - x
            dy = obs['y'] - y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist <= radius and (label is None or obs['label'] == label):
                total += dist
        return total
    # sum function maybe useless since when domain is straight line, function is convex => not useful information for task goal setting
    
    def weighted_obstacle_distance(self, x: float, y: float, label: str = None, lam: float = 1.0) -> float:
        total = 0.0
        for key, obs in self.obstacles.items():
            if label is not None and obs['label'] != label:
                continue
            dx = obs['x'] - x
            dy = obs['y'] - y
            dist = (dx * dx + dy * dy) ** 0.5
            total += math.exp(-(1/obs['radius']) * dist)
        return total
    # cost = e^(-lamda * distance) is exponential decay where as lambda increases, rate of decay increases
    
    def min_cost_on_line(self, x1: float, y1: float, x2: float, y2: float, label: str = None, size: float = 10) -> float:
        t = 0
        min = math.inf
        dx = x2 - x1
        dy = y2 - y1
        for i in range(size + 1):
            temp_min = self.weighted_obstacle_distance(x1 + i * (1/size) * dx, y1 + i * (1/size) * dy)
            if temp_min < min:
                min = temp_min
                t = i * (1/size)
        return t 
    # size = size of sample along the line. 100 size means 100 points will be taken, evenly spaced across the line
    # t is a multiple of the line where t * (direction of line) = a point on the line


    def mark_used(self, key: str):
        if key in self.obstacles:
            self.obstacles[key]['used'] = True
            self.save_to_disk()

    def get_all(self) -> dict:
        return dict(self.obstacles)

    def publish_markers(self):
        array = MarkerArray()
        for key, obs in self.obstacles.items():
            stamp = self.get_clock().now().to_msg()
            color = self._color_for_label(obs['label'])
            marker_id = abs(hash(key)) % 2000000000
            circle = Marker()
            circle.header.frame_id = 'map'
            circle.header.stamp = stamp
            circle.ns = 'obstacle_type_circles'
            circle.id = marker_id
            circle.type = Marker.CYLINDER
            circle.action = Marker.ADD
            circle.pose.position.x = obs['x']
            circle.pose.position.y = obs['y']
            circle.pose.position.z = 0.0
            circle.pose.orientation.w = 1.0
            circle.scale.x = obs['radius'] * 2.0
            circle.scale.y = obs['radius'] * 2.0
            circle.scale.z = 0.01
            circle.color = color
            array.markers.append(circle)
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = stamp
            text.ns = 'obstacle_type_text'
            text.id = marker_id
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
        self.marker_pub.publish(array)

    def save_to_disk(self):
        try:
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            with open(self.save_path, 'w') as f:
                json.dump(self.obstacles, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f'Failed to save obstacle type map: {e}')

    def _color_for_label(self, label: str) -> ColorRGBA:
        LABEL_COLORS = {
            "green_pole_buoy": ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.85),
            "red_pole_buoy":   ColorRGBA(r=0.9, g=0.0, b=0.0, a=0.85),
        }
        if label in LABEL_COLORS:
            return LABEL_COLORS[label]
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