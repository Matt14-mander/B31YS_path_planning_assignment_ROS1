#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower')

        self.path_sub = rospy.Subscriber('/planned_path', Path, self.path_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.listener = tf.TransformListener()

        self.path = []
        self.current_target_idx = 0
        self.waypoint_tolerance = 0.2  # meters
        self.angular_kp = 1.5
        self.linear_kp = 0.3

        self.rate = rospy.Rate(10)
        rospy.loginfo("Path follower initialized.")
        start_delay = rospy.get_param("~start_delay", 0.0)
        rospy.sleep(start_delay)

    def path_callback(self, msg):
        self.path = msg.poses
        self.current_target_idx = 0
        rospy.loginfo("Received new path with %d points", len(self.path))

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def run(self):
        while not rospy.is_shutdown():
            if not self.path or self.current_target_idx >= len(self.path):
                # No path or done
                self.cmd_pub.publish(Twist())  # stop
                self.rate.sleep()
                continue

            try:
                (trans, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("TF lookup failed")
                self.rate.sleep()
                continue

            robot_x, robot_y = trans[0], trans[1]
            (_, _, robot_yaw) = tf.transformations.euler_from_quaternion(rot)

            target_pose = self.path[self.current_target_idx].pose.position
            dx = target_pose.x - robot_x
            dy = target_pose.y - robot_y
            distance = math.hypot(dx, dy)

            angle_to_goal = math.atan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_goal - robot_yaw)

            cmd = Twist()

            if distance > self.waypoint_tolerance:
                # Proportional control for angular velocity
                cmd.angular.z = self.angular_kp * angle_error

                # Move forward only if facing roughly towards goal
                if abs(angle_error) < 0.3:
                    cmd.linear.x = min(self.linear_kp * distance, 0.3)
                else:
                    cmd.linear.x = 0.0
            else:
                rospy.loginfo("Reached waypoint %d", self.current_target_idx)
                self.current_target_idx += 1
                cmd = Twist()  # stop momentarily at waypoint

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = PathFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass

