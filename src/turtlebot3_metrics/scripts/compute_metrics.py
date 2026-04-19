#!/usr/bin/env python
"""
Compute navigation metrics from TurtleBot3 metrics CSV.

Metrics reported:
- Goal Reach Rate
- Collision Count
- Path Optimality
- Re-planning Speed
"""

import argparse
import csv
import json
import math


class MetricsComputer:
    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.rows = self._load_csv()
        self.metrics = {}

    def _load_csv(self):
        rows = []
        with open(self.csv_file, "r") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                row["time"] = float(row["time"])
                row["x"] = float(row["x"])
                row["y"] = float(row["y"])
                row["attempt_id"] = int(float(row["attempt_id"]))
                row["goal_x"] = float(row["goal_x"]) if row["goal_x"] not in ("", "nan") else float("nan")
                row["goal_y"] = float(row["goal_y"]) if row["goal_y"] not in ("", "nan") else float("nan")
                row["goal_reached_count"] = int(float(row["goal_reached_count"]))
                row["collision_count"] = int(float(row["collision_count"]))
                row["plan_update_count"] = int(float(row["plan_update_count"]))
                row["last_plan_update_time"] = (
                    float(row["last_plan_update_time"])
                    if row["last_plan_update_time"] not in ("", "nan")
                    else float("nan")
                )
                rows.append(row)
        return rows

    @staticmethod
    def _distance(x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def compute_goal_reach_rate(self):
        attempts = sorted({row["attempt_id"] for row in self.rows if row["attempt_id"] > 0})
        succeeded_attempts = 0

        for attempt_id in attempts:
            attempt_rows = [row for row in self.rows if row["attempt_id"] == attempt_id]
            statuses = {row["goal_status"] for row in attempt_rows}
            if "SUCCEEDED" in statuses:
                succeeded_attempts += 1

        total_attempts = len(attempts)
        goal_reach_rate = float(succeeded_attempts) / float(total_attempts) if total_attempts else 0.0

        self.metrics["goal_reach_rate"] = goal_reach_rate
        self.metrics["goal_reach_rate_percent"] = goal_reach_rate * 100.0
        self.metrics["successful_goals"] = succeeded_attempts
        self.metrics["total_goal_attempts"] = total_attempts

    def compute_collision_count(self):
        collision_count = max((row["collision_count"] for row in self.rows), default=0)
        self.metrics["collision_count"] = collision_count

    def compute_path_optimality(self):
        attempts = sorted({row["attempt_id"] for row in self.rows if row["attempt_id"] > 0})
        optimality_scores = []

        for attempt_id in attempts:
            attempt_rows = [row for row in self.rows if row["attempt_id"] == attempt_id]
            if len(attempt_rows) < 2:
                continue

            statuses = {row["goal_status"] for row in attempt_rows}
            if "SUCCEEDED" not in statuses:
                continue

            start_row = attempt_rows[0]
            goal_rows = [row for row in attempt_rows if not math.isnan(row["goal_x"]) and not math.isnan(row["goal_y"])]
            if not goal_rows:
                continue

            goal_row = goal_rows[-1]
            straight_line = self._distance(start_row["x"], start_row["y"], goal_row["goal_x"], goal_row["goal_y"])

            path_length = 0.0
            for prev_row, cur_row in zip(attempt_rows[:-1], attempt_rows[1:]):
                path_length += self._distance(prev_row["x"], prev_row["y"], cur_row["x"], cur_row["y"])

            if path_length <= 1e-6:
                continue

            optimality_scores.append(straight_line / path_length if straight_line > 0 else 0.0)

        self.metrics["path_optimality"] = sum(optimality_scores) / len(optimality_scores) if optimality_scores else 0.0
        self.metrics["path_optimality_samples"] = len(optimality_scores)

    def compute_replanning_speed(self):
        update_times = []
        last_time = None

        for row in self.rows:
            current_time = row["last_plan_update_time"]
            if math.isnan(current_time):
                continue
            if last_time is None or abs(current_time - last_time) > 1e-6:
                update_times.append(current_time)
                last_time = current_time

        if len(update_times) < 2:
            self.metrics["replanning_speed_hz"] = 0.0
            self.metrics["avg_replanning_interval_s"] = 0.0
            self.metrics["replanning_updates"] = len(update_times)
            return

        intervals = [cur - prev for prev, cur in zip(update_times[:-1], update_times[1:]) if cur > prev]
        if not intervals:
            self.metrics["replanning_speed_hz"] = 0.0
            self.metrics["avg_replanning_interval_s"] = 0.0
            self.metrics["replanning_updates"] = len(update_times)
            return

        avg_interval = sum(intervals) / len(intervals)
        self.metrics["avg_replanning_interval_s"] = avg_interval
        self.metrics["replanning_speed_hz"] = 1.0 / avg_interval if avg_interval > 0 else 0.0
        self.metrics["replanning_updates"] = len(update_times)

    def compute_all(self):
        self.compute_goal_reach_rate()
        self.compute_collision_count()
        self.compute_path_optimality()
        self.compute_replanning_speed()
        return self.metrics

    def save_metrics(self, output_file):
        with open(output_file, "w") as handle:
            json.dump(self.metrics, handle, indent=2)
        print("[SUCCESS] Metrics saved to {}".format(output_file))

    def print_metrics(self):
        print("\n" + "=" * 60)
        print("NAVIGATION METRICS")
        print("=" * 60)
        print("Goal Reach Rate       : {:.2f}%".format(self.metrics["goal_reach_rate_percent"]))
        print("Collision Count       : {}".format(self.metrics["collision_count"]))
        print("Path Optimality       : {:.4f}".format(self.metrics["path_optimality"]))
        print("Re-planning Speed     : {:.4f} Hz".format(self.metrics["replanning_speed_hz"]))
        print("Avg Replan Interval   : {:.4f} s".format(self.metrics["avg_replanning_interval_s"]))
        print("=" * 60 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Compute TurtleBot3 navigation metrics")
    parser.add_argument("csv_file", help="Metrics CSV file")
    parser.add_argument("-o", "--output", default=None, help="Output JSON file")
    args = parser.parse_args()

    output_file = args.output or args.csv_file.replace(".csv", "_metrics.json")

    computer = MetricsComputer(args.csv_file)
    computer.compute_all()
    computer.print_metrics()
    computer.save_metrics(output_file)


if __name__ == "__main__":
    main()
