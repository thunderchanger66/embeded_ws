#!/usr/bin/env python3
# coding=utf-8
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

Moving arm:
   1    2    3   4   5   6
   q    w    e   r   t   y

a/z : increase/decrease max speeds by 10%
s/x : increase/decrease only linear speed by 10%
d/c : increase/decrease precision by 0.05
space key : reset
k : force stop
f : special position
anything else : stop smoothly
b : switch to OmniMode/CommonMode
precision is not less than or equal to zero
CTRL-C to quit
"""

Omni = 0
precision = 0.05

rotateBindings = {
    '1': (1, 1), 'q': (1, -1),
    '2': (2, 1), 'w': (2, -1),
    '3': (3, 1), 'e': (3, -1),
    '4': (4, 1), 'r': (4, -1),
    '5': (5, 1), 't': (5, -1),
    '6': (6, 1), 'y': (6, -1),
}

precisionBindings = {
    'd': 0.01,
    'c': -0.01
}

moveBindings = {
    'i': (1, 0), 'o': (1, -1),
    'j': (0, 1), 'l': (0, -1),
    'u': (1, 1), ',': (-1, 0),
    '.': (-1, 1), 'm': (-1, -1)
}

speedBindings = {
    'a': (1.1, 1), 'z': (0.9, 1),
    's': (1, 1.1), 'x': (1, 0.9)
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def prec(speed, turn, precision):
    return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}\tprecision {precision:.3f}"

def main():
    global precision
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('arm_teleop')

    pub_arm = node.create_publisher(JointState, '/joint_states', 5)
    pub_vel = node.create_publisher(Twist, '/cmd_vel', 5)

    joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    jointState = JointState()
    jointState.name = [f"joint{i+1}" for i in range(6)]
    jointState.position = joints

    twist = Twist()
    x = th = count = 0
    speed = 0.2
    turn = 1.0
    target_speed = target_turn = target_HorizonMove = 0.0
    control_speed = control_turn = control_HorizonMove = 0.0

    try:
        print(msg)
        print(prec(speed, turn, precision))

        while rclpy.ok():
            key = getKey(settings)

            global Omni
            if key == 'b':
                Omni = ~Omni
                print("Switch to OmniMode" if Omni else "Switch to CommonMode")
                moveBindings['.'], moveBindings['m'] = ((-1, -1), (-1, 1)) if Omni else ((-1, 1), (-1, -1))

            if key in moveBindings:
                x, th = moveBindings[key]
                count = 0
            elif key in speedBindings:
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
                print(prec(speed, turn, precision))
                count = 0
            elif key == 'k':
                x = th = control_speed = control_turn = 0
                count = 0
            elif key in rotateBindings:
                count = 0
                idx, sign = rotateBindings[key]
                joints[idx-1] += precision * sign
                joints[idx-1] = max(min(joints[idx-1], 1.57), -1.57)
            elif key in precisionBindings:
                if 0 < precision + precisionBindings[key] <= 0.1:
                    precision += precisionBindings[key]
                    print(prec(speed, turn, precision))
                count = 0
            elif key == ' ':
                joints = [0.0]*6
                count = 0
            elif key == 'f':
                joints = [0.0, 0.57, 1.57, 1.3, 0.0, 0.6]
                count = 0
            elif key == '\x03':
                break
            else:
                count += 1
                if count > 4:
                    x = th = 0

            target_speed = speed * x
            target_turn = turn * th
            target_HorizonMove = speed * th

            control_speed += 0.1 * ((target_speed > control_speed) - (target_speed < control_speed))
            control_turn += 0.5 * ((target_turn > control_turn) - (target_turn < control_turn))
            control_HorizonMove += 0.1 * ((target_HorizonMove > control_HorizonMove) - (target_HorizonMove < control_HorizonMove))

            twist.linear.x = control_speed
            twist.linear.y = control_HorizonMove if Omni else 0.0
            twist.linear.z = 0.0
            twist.angular.z = control_turn if not Omni else 0.0

            pub_vel.publish(twist)

            jointState.header.stamp = node.get_clock().now().to_msg()
            jointState.position = joints
            pub_arm.publish(jointState)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub_vel.publish(twist)
        print("Keyboard control off")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
