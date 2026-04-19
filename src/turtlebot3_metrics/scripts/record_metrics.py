#!/usr/bin/env python
"""
Record navigation metrics for TurtleBot3.

Tracked signals:
- robot odometry
- goal attempts and completion
- collision events from LaserScan thresholding
- TEB local plan update timestamps
"""

import argparse
import csv
import math

import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


class MetricsRecorder:
    def __init__(self, output_file):
        self.output_file = output_file
        self.csv_file = open(output_file, "w")
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "time",
                "x",
                "y",
                "vx",
                "vy",
                "omega",
                "linear_velocity",
                "goal_status",
                "attempt_id",
                "goal_x",
                "goal_y",
                "goal_reached_count",
                "collision_count",
                "min_scan_range",
                "plan_update_count",
                "last_plan_update_time",
            ],
        )
        self.csv_writer.writeheader()

        self.goal_status = "IDLE"
        self.last_status_code = None
        self.start_time = rospy.Time.now()
        self.collision_distance = rospy.get_param("~collision_distance", 0.22)

        self.attempt_id = 0
        self.goal_reached_count = 0
        self.collision_count = 0
        self.plan_update_count = 0

        self.current_goal_x = float("nan")
        self.current_goal_y = float("nan")
        self.last_goal_signature = None
        self.last_plan_update_time = float("nan")

        self.collision_active = False
        self.min_scan_range = float("inf")

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=20)
        self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback, queue_size=10)
        self.goal_sub = rospy.Subscriber("/move_base/current_goal", PoseStamped, self.goal_callback, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=20)
        self.plan_sub = rospy.Subscriber(
            "/move_base/TebLocalPlannerROS/local_plan", Path, self.local_plan_callback, queue_size=20
        )

        print("[INFO] Recording metrics to {}".format(output_file))

    def elapsed_time(self):
        return (rospy.Time.now() - self.start_time).to_sec()

    def goal_callback(self, msg):
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        signature = (round(goal_x, 3), round(goal_y, 3))

        if signature != self.last_goal_signature:
            self.attempt_id += 1
            self.last_goal_signature = signature

        self.current_goal_x = goal_x
        self.current_goal_y = goal_y

    def local_plan_callback(self, _msg):
        self.plan_update_count += 1
        self.last_plan_update_time = self.elapsed_time()

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self.min_scan_range = min(valid_ranges) if valid_ranges else float("inf")

        collision_now = self.min_scan_range <= self.collision_distance
        if collision_now and not self.collision_active:
            self.collision_count += 1
        self.collision_active = collision_now

    def status_callback(self, msg):
        if not msg.status_list:
            return

        latest_status = msg.status_list[-1].status
        if latest_status == self.last_status_code:
            return

        self.last_status_code = latest_status
        if latest_status == 1:
            self.goal_status = "ACTIVE"
        elif latest_status == 3:
            self.goal_status = "SUCCEEDED"
            self.goal_reached_count += 1
        elif latest_status == 4:
            self.goal_status = "ABORTED"
        elif latest_status == 2:
            self.goal_status = "PREEMPTED"
        else:
            self.goal_status = "IDLE"

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        linear_velocity = math.sqrt(vx ** 2 + vy ** 2)

        self.csv_writer.writerow(
            {
                "time": self.elapsed_time(),
                "x": x,
                "y": y,
                "vx": vx,
                "vy": vy,
                "omega": omega,
                "linear_velocity": linear_velocity,
                "goal_status": self.goal_status,
                "attempt_id": self.attempt_id,
                "goal_x": self.current_goal_x,
                "goal_y": self.current_goal_y,
                "goal_reached_count": self.goal_reached_count,
                "collision_count": self.collision_count,
                "min_scan_range": self.min_scan_range,
                "plan_update_count": self.plan_update_count,
                "last_plan_update_time": self.last_plan_update_time,
            }
        )
        self.csv_file.flush()

    def stop(self):
        self.csv_file.close()
        print("[SUCCESS] Metrics saved to {}".format(self.output_file))


def main():
    parser = argparse.ArgumentParser(description="Record TurtleBot3 navigation metrics")
    parser.add_argument("-o", "--output", default=None, help="Output CSV file")
    args, _ = parser.parse_known_args()

    rospy.init_node("metrics_recorder", anonymous=True)
    output_file = args.output or rospy.get_param("~output_file", "metrics.csv")
    recorder = MetricsRecorder(output_file)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\n[INFO] Stopping recording...")
    finally:
        recorder.stop()


if __name__ == "__main__":
    main()
