#!/usr/bin/env python3


import rclpy

from turtlebot3_position_conntrol import Turtlebot3PositionControl


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_position_control = Turtlebot3PositionControl()
    rclpy.spin(turtlebot3_position_control)

    turtlebot3_position_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
