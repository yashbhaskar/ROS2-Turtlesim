import rclpy
import subprocess
from rclpy.node import Node

# Dummy node just for initialization and shutdown
class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_launcher')

def main():
    rclpy.init()
    node = ShapeNode()

    while True:
        print("Which shape do you want to draw? (square/star/triangle/circle/12star/hexagon/octagon/clear): ")
        shape = input().strip().lower()

        if shape not in ['square', 'star', 'triangle', 'circle', '12star', 'hexagon', 'octagon', 'clear']:
            print("Invalid shape. Please choose a valid option.")
            continue

        if shape == 'square':
            executable = 'square'
        elif shape == 'star':
            executable = 'star'
        elif shape == 'triangle':
            executable = 'triangle'
        elif shape == 'circle':
            executable = 'circle'
        elif shape == '12star':
            executable = 'star12'
        elif shape == 'hexagon':
            executable = 'hexagon'
        elif shape == 'octagon':
            executable = 'octagon'
        elif shape == 'clear':
            executable = 'clear'
    
        cmd = ['ros2', 'run', 'shapes_pkg', executable]
        subprocess.run(cmd)
        rclpy.shutdown()
        
        inp = input("Do you want to draw again? (yes/no): ").strip().lower()
        if inp != 'yes':
            break
        else:
            rclpy.init()
            node = ShapeNode()

if __name__ == '__main__':
    main()
