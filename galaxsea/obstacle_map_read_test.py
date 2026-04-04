import rclpy
from galaxsea.obstacle_type_map import ObstacleTypeMapNode
import threading


def input_loop(node):
    print("\nObstacle Type Map Lookup Test")
    print("Format: x y <optional_radius (default 0.5)>")
    print("Commands: list, quit\n")

    while True:
        try:
            user_input = input("> ").strip()

            if user_input == 'quit':
                break
            elif user_input == 'list':
                all_obs = node.get_all()
                if not all_obs:
                    print("No obstacles stored.")
                else:
                    for key, obs in all_obs.items():
                        print(f"  [{key}] '{obs['label']}' at ({obs['x']:.2f}, {obs['y']:.2f}) radius={obs['radius']:.2f} count={obs['count']}")
            else:
                parts = user_input.split()
                if len(parts) == 2:
                    x, y = float(parts[0]), float(parts[1])
                    node.load_from_disk()
                    results = node.lookup(x, y)
                    if not results:
                        print(f"  No obstacle found at ({x}, {y})")
                    else:
                        print(f"  Found {len(results)} obstacle(s):")
                        for obs in results:
                            print(f"    '{obs['label']}' at ({obs['x']:.2f}, {obs['y']:.2f}) dist={obs['distance']:.3f} radius={obs['radius']:.2f} count={obs['count']}")
                else:
                    print("Invalid format. Use: x y")
        except ValueError:
            print("Invalid x, y, or radius — must be numbers.")
        except KeyboardInterrupt:
            break


def main():
    rclpy.init()
    node = ObstacleTypeMapNode()

    thread = threading.Thread(target=input_loop, args=(node,), daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()