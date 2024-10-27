import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import numpy as np
from sklearn.linear_model import RANSACRegressor, LinearRegression

from functools import wraps
import time

import matplotlib.pyplot as plt


def timeit(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        print(f'Function {func.__name__} Took {total_time:.4f} seconds, {1/total_time:.4f} Hz')
        return result
    return timeit_wrapper


class LaserScan2Lines(Node):

    def __init__(self):
        super().__init__('laserscan2lines')

        self.create_subscription(LaserScan, "scan", self.callback, 10)

    @timeit
    def callback(self, msg: LaserScan):
        # Convert LaserScan ranges to Cartesian coordinates
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges (e.g., values at infinity)
        valid_indices = np.isfinite(ranges)
        angles = angles[valid_indices]
        ranges = ranges[valid_indices]

        # Calculate x and y coordinates
        x_points = ranges * np.cos(angles)
        y_points = ranges * np.sin(angles)
        points = np.vstack((x_points, y_points)).T

        lines = []  # To store detected lines
        line_equations = []  # To store line equations

        # Detect multiple lines using iterative RANSAC
        min_points_for_line = 50  # Minimum points to define a line
        ransac = RANSACRegressor(residual_threshold=0.1)

        while len(points) > min_points_for_line:
            # Fit RANSAC model to the remaining points
            ransac.fit(points[:, 0].reshape(-1, 1), points[:, 1])

            # Extract inliers and outliers
            inlier_mask = ransac.inlier_mask_
            line_points = points[inlier_mask]

            # Save the detected line
            if len(line_points) >= min_points_for_line:
                lines.append(line_points)

                # Fit a linear regression model to the inlier points to get the line equation
                line_model = LinearRegression()
                line_model.fit(line_points[:, 0].reshape(-1, 1), line_points[:, 1])

                # Get line equation parameters
                slope = line_model.coef_[0]   # m in y = mx + c
                intercept = line_model.intercept_  # c in y = mx + c
                line_equations.append((slope, intercept))
                self.get_logger().info(f"Detected line with {len(line_points)} points, [y = {slope:.2f}x + {intercept:.2f}]")

            # Remove inlier points and continue
            points = points[~inlier_mask]
        
        # Plot the points and lines
        # self.plot_lines_and_points(lines, line_equations)

    def plot_lines_and_points(self, lines, line_equations):
        plt.figure(figsize=(10, 8))
        
        # Plot each line's points
        for idx, line_points in enumerate(lines):
            plt.scatter(line_points[:, 0], line_points[:, 1], label=f'Line {idx+1} Points')

            # Plot the fitted line
            slope, intercept = line_equations[idx]
            x_vals = np.linspace(np.min(line_points[:, 0]), np.max(line_points[:, 0]), 100)
            y_vals = slope * x_vals + intercept
            plt.plot(x_vals, y_vals, label=f'Line {idx+1}: y = {slope:.2f}x + {intercept:.2f}', linewidth=2)

        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.title("Detected Lines from LaserScan Data")
        plt.legend()
        plt.grid()
        plt.show()

def main():
    rclpy.init()
    node = LaserScan2Lines()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # cleans up pub-subs, etc
        rclpy.try_shutdown()     

if __name__ == "__main__":
    main()