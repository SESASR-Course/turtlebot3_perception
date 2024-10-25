import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rclpy.time import Time
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from landmark_interfaces.msg import LandmarkArray, Landmark

import math

class MyNode(Node):

    def __init__(self):
        super().__init__('my_node_name')

        self.parent_frame = self.declare_parameter('parent_frame', 'base_link').get_parameter_value().string_value

        self.landmark_array_pub = self.create_publisher(LandmarkArray, "landmarks", 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(AprilTagDetectionArray, "detections", self.callback, 10)

    def callback(self, msg: AprilTagDetectionArray):
        time = Time()
        landmarks_msg = LandmarkArray()
        landmarks_msg.header = msg.header
        landmarks_msg.header.frame_id = self.parent_frame
        for tag in msg.detections:
            landmark = Landmark()
            landmark.hamming = tag.hamming
            landmark.goodness = tag.goodness
            landmark.decision_margin = tag.decision_margin
            target_frame = f"{tag.family}:{tag.id}"
            landmark.id = target_frame
            tf = self.buffer.lookup_transform(self.parent_frame, target_frame, time)
            x, y, z = tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
            landmark.range = math.sqrt(x**2 + y**2 + z**2)
            landmark.bearing = math.atan2(y, x)
            landmarks_msg.landmarks.append(landmark)

        self.landmark_array_pub.publish(landmarks_msg)
        

def main():
    rclpy.init()
    my_node = MyNode()
    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_node.destroy_node()  # cleans up pub-subs, etc
        rclpy.try_shutdown()     

if __name__ == "__main__":
    main()