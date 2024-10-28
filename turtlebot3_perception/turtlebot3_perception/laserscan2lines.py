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

@timeit
def laserscan2lines(scan: LaserScan):
    """
    A function to detect lines from a LaserScan message using RANSAC.
    A normal line is represented by y = mx + c, where m is the slope and c is the y-intercept.
    If the line is vertical (angle >85°), the line equation is x = c, where c is the x-intercept.

    :param scan: A sensor_msgs/LaserScan message
    :return: A list of detected lines, where each line is a list of points (x, y) and the line equation (slope, intercept). If the line is vertical, the slope is infinity.
    """

    # Convert LaserScan ranges to Cartesian coordinates
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    ranges = np.array(scan.ranges)

    # Filter out invalid ranges (e.g., values at infinity)
    valid_indices = np.isfinite(ranges) & (ranges < scan.range_max) & (ranges > scan.range_min)
    angles = angles[valid_indices]
    ranges = ranges[valid_indices]

    # Calculate x and y coordinates
    x_points = ranges * np.cos(angles)
    y_points = ranges * np.sin(angles)
    points = np.vstack((x_points, y_points)).T

    lines = []  # To store detected lines
    line_equations = []  # To store line equations

    # Detect multiple lines using iterative RANSAC
    min_points_for_line = 30  # Minimum points to define a line
    ransac = RANSACRegressor(residual_threshold=0.1)

    while len(points) > min_points_for_line: 
        with np.errstate(divide='ignore'):
            stDevX = np.std(points[:, 0]) 
            stDevY = np.std(points[:, 1])
            verticality = stDevY/stDevX

        if verticality < 15:
            # Fit RANSAC model to the remaining points
            ransac.fit(points[:, 0].reshape(-1, 1), points[:, 1])
            line_model = ransac.estimator_
            slope = line_model.coef_[0]   # m in y = mx + c
            intercept = line_model.intercept_  # c in y = mx + c
        else:
            ransac.fit(points[:, 1].reshape(-1, 1), points[:, 0])
            line_model = ransac.estimator_
            slope = np.inf
            intercept = line_model.intercept_  # c in x = c

        # Extract inliers and outliers
        inlier_mask = ransac.inlier_mask_
        line_points = points[inlier_mask]

        # Save the detected line
        if len(line_points) >= min_points_for_line:
            lines.append(line_points)

            # Get line equation parameters
            line_equations.append((slope, intercept))
        # Remove inlier points and continue
        points = points[~inlier_mask]

    
    return lines, line_equations



class LaserScan2Lines(Node):

    def __init__(self):
        super().__init__('laserscan2lines')

        self.create_subscription(LaserScan, "scan", self.callback, 10)
        plt.figure(figsize=(8, 5))
        plt.show(block=False)
        self.plot_locked = False


    def callback(self, msg: LaserScan):

        lines, line_equations = laserscan2lines(msg)

        self.get_logger().info(f"Toatal lines detected {len(lines)}")
        for line_points, (slope, intercept) in zip(lines, line_equations):
            self.get_logger().info(f"Detected line with {len(line_points)} points, [y = {slope:.2f}x + {intercept:.2f}]")

        
        # Plot the points and lines
        self.plot_lines_and_points(lines, line_equations)

    def plot_lines_and_points(self, lines, line_equations):        
        # Plot each line's points
        if self.plot_locked:
            return
        
        self.plot_locked = True
        plt.cla()
        for idx, line_points in enumerate(lines):
            plt.scatter(line_points[:, 0], line_points[:, 1], label=f'Line {idx+1} Points')

            # Plot the fitted line
            slope, intercept = line_equations[idx]
            if slope == np.inf:
                # vertical lines (x = c)
                x_vals = np.full(100, intercept)
                y_vals = np.linspace(np.min(line_points[:, 1]), np.max(line_points[:, 1]), 100)
                plt.plot(x_vals, y_vals, label=f'Line {idx+1}: x = {intercept:.2f}', linewidth=2)
            else:
                # normal lines (y = mx + c)
                x_vals = np.linspace(np.min(line_points[:, 0]), np.max(line_points[:, 0]), 100)
                y_vals = slope * x_vals + intercept
                plt.plot(x_vals, y_vals, label=f'Line {idx+1}: y = {slope:.2f}x + {intercept:.2f}', linewidth=2)
        plt.xlim(-3.5, 3.5)
        plt.ylim(-3.5, 3.5)
        plt.axis('equal')
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.title("Detected Lines from LaserScan Data")
        plt.legend()
        plt.grid()
        plt.draw()
        plt.pause(0.005)
        self.plot_locked = False

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