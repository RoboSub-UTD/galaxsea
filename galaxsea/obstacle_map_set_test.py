import rclpy
from galaxsea.obstacle_type_map import ObstacleTypeMapNode
import threading


def input_loop(node):
    print("\nObstacle Type Map Tester")
    print("Format: x y label <optional_radius>")
    print("Commands: list, clear, quit\n")

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
                        print(f"  {obs['label']} at ({obs['x']:.2f}, {obs['y']:.2f}) radius={obs['radius']:.2f}")
            elif user_input == 'clear':
                node.obstacles = {}
                node.save_to_disk()
                print("Cleared all obstacles.")
            else:
                parts = user_input.split()
                if len(parts) == 4:
                    x, y, label, radius = float(parts[0]), float(parts[1]), parts[2], float(parts[3])
                    key = node.add_obstacle(x, y, label, radius=radius)
                    print(f"Added '{label}' at ({x}, {y}) radius={radius} key={key}")
                elif len(parts) == 3:
                    x, y, label = float(parts[0]), float(parts[1]), parts[2]
                    key = node.add_obstacle(x, y, label)
                    print(f"Added '{label}' at ({x}, {y}) key={key}")
                else:
                    print("Invalid format. Use: x y label  or  x y label radius")

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