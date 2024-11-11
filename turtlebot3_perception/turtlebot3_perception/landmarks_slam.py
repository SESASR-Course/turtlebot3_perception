import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_py as tf2

from landmark_msgs.msg import LandmarkArray

import ruamel.yaml
import numpy as np


#####################################################
# USED FOR PRETTY YAML OUTPUT
def seq(*l):
    s = ruamel.yaml.comments.CommentedSeq(l)
    s.fa.set_flow_style()
    return s


#####################################################


class LandmarkSLAM(Node):

    def __init__(self):
        super().__init__("landmark_slam")

        self.map_frame = self.declare_parameter("map_frame", "map").get_parameter_value().string_value

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.subscription = self.create_subscription(LandmarkArray, "landmarks", self.landmark_callback, 10)
        self.landmarks = set()
        self.landmarks_positions = {}
        self.landmarks_positions_latest = {}

        self.create_timer(0.1, self.update_landmarks_position)

    def update_landmarks_position(self):
        for lmark_id in self.landmarks:
            try:
                tf = self.buffer.lookup_transform(self.map_frame, f"landmark_{lmark_id}", Time())
                x, y, z = tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
                if not lmark_id in self.landmarks_positions:
                    self.landmarks_positions[lmark_id] = []
                self.landmarks_positions[lmark_id].append((x, y, z))
                self.landmarks_positions_latest[lmark_id] = (x, y, z)
            except tf2.TransformException:
                continue

    def landmark_callback(self, msg: LandmarkArray):
        for lmark in msg.landmarks:
            self.landmarks.add(lmark.id)

    def on_shutdown(self):
        for lmark_id in self.landmarks_positions:
            x, y, z = np.median(self.landmarks_positions[lmark_id], axis=0)
            self.landmarks_positions[lmark_id] = (x, y, z)

        self.landmarks_positions = dict(sorted(self.landmarks_positions.items()))
        self.landmarks_positions_latest = dict(sorted(self.landmarks_positions_latest.items()))

        # OUTPUT AS TEXT FILE
        # with open("landmarks.txt", "w") as f:
        #     f.write("Detected landmarks...\n")
        #     f.write("ID\tX\tY\tZ\n")
        #     for lmark_id, (x, y, z) in self.landmarks_positions.items():
        #         f.write(f"{lmark_id}\t{x:.2f}\t{y:.2f}\t{z:.2f}\n")

        # OUTPUS AS YAML PARAMETER
        self.landmarks_out = {"landmarks": {}}
        self.landmarks_out["landmarks"]["id"] = seq()
        self.landmarks_out["landmarks"]["x"] = seq()
        self.landmarks_out["landmarks"]["y"] = seq()
        self.landmarks_out["landmarks"]["z"] = seq()
        for lmark_id, (x, y, z) in self.landmarks_positions.items():
            self.landmarks_out["landmarks"]["id"].append(lmark_id)
            self.landmarks_out["landmarks"]["x"].append(round(float(x), 3))
            self.landmarks_out["landmarks"]["y"].append(round(float(y), 3))
            self.landmarks_out["landmarks"]["z"].append(round(float(z), 3))
        with open("landmarks.yaml", "w") as f:
            # for l in self.landmarks_out["landmarks"]:
            #     self.landmarks_out["landmarks"][l] = seq(*self.landmarks_out["landmarks"][l])
            yaml = ruamel.yaml.YAML()
            yaml.default_flow_style = False
            yaml.dump(self.landmarks_out, f)

        self.landmarks_out = {"landmarks": {}}
        self.landmarks_out["landmarks"]["id"] = seq()
        self.landmarks_out["landmarks"]["x"] = seq()
        self.landmarks_out["landmarks"]["y"] = seq()
        self.landmarks_out["landmarks"]["z"] = seq()
        for lmark_id, (x, y, z) in self.landmarks_positions_latest.items():
            self.landmarks_out["landmarks"]["id"].append(lmark_id)
            self.landmarks_out["landmarks"]["x"].append(round(float(x), 3))
            self.landmarks_out["landmarks"]["y"].append(round(float(y), 3))
            self.landmarks_out["landmarks"]["z"].append(round(float(z), 3))
        with open("landmarks_latest.yaml", "w") as f:
            # for l in self.landmarks_out["landmarks"]:
            #     self.landmarks_out["landmarks"][l] = seq(*self.landmarks_out["landmarks"][l])
            yaml = ruamel.yaml.YAML()
            yaml.default_flow_style = False
            yaml.dump(self.landmarks_out, f)


def main():
    rclpy.init()
    my_node = LandmarkSLAM()
    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_node.on_shutdown()  # do any custom cleanup
        my_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
