"""
hsv_tuner - interaktivt verktoey for aa finne HSV-terskler per kubefarge.

Abonnerer paa kamera-stroemmen, viser live-bilde + binaer maske + maskert
resultat side om side, og lar deg dra H/S/V-grenser med OpenCV-trackbars
til kuben isoleres rent. Trykk 's' for aa lagre gjeldende verdier til
hsv_thresholds.yaml under fargen gitt av `color`-parameteren. Trykk 'q'
eller ESC for aa avslutte.

Opphav per blokk:
  - rclpy-node + cv_bridge image-subscriber: moenster fra Lab4
    camera_pipeline (AIS2105).
  - BGR->HSV med cvtColor + inRange-maskering: OpenCV "Changing
    Colorspaces"-tutorial
    (https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html).
  - Trackbar-GUI, lagring til YAML og keypress-haandtering: generert
    av Claude.
  - Note om at roed farge maa tunes i to omganger (H wrapper rundt
    0/180): fra testlogg.md test 20260512-09.

Bruk (med kamera kjoerende paa /image_raw):
    ros2 run vision hsv_tuner --ros-args -p color:=blue \\
        -p output_file:=src/vision/config/hsv_thresholds.yaml
"""
import os

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

WINDOW = 'hsv_tuner'
TRACKBARS = ['H low', 'H high', 'S low', 'S high', 'V low', 'V high']
# OpenCV HSV-rekkevidde: H 0-179, S/V 0-255.
TRACKBAR_MAX = {'H low': 179, 'H high': 179,
                'S low': 255, 'S high': 255,
                'V low': 255, 'V high': 255}
# Fornuftige startposisjoner saa vinduet ikke aapner helt svart.
TRACKBAR_INIT = {'H low': 0, 'H high': 179,
                 'S low': 80, 'S high': 255,
                 'V low': 80, 'V high': 255}


class HsvTuner(Node):
    """Live HSV-trackbar-tuner som lagrer terskler til YAML."""

    def __init__(self):
        super().__init__('hsv_tuner')

        # Parametere - gjoer noden konfigurerbar uten kodeendring
        # (teller paa ROS2-kategorien "konfigurerbart").
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('color', 'red')
        self.declare_parameter('output_file', 'hsv_thresholds.yaml')

        image_topic = self.get_parameter('image_topic').value
        self.color = self.get_parameter('color').value
        self.output_file = self.get_parameter('output_file').value

        # cv_bridge-subscriber: moenster fra Lab4 camera_pipeline.
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        # OpenCV-vindu med seks trackbars. Generert.
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        for name in TRACKBARS:
            cv2.createTrackbar(name, WINDOW, TRACKBAR_INIT[name],
                               TRACKBAR_MAX[name], lambda _v: None)

        self.get_logger().info(
            f"hsv_tuner i gang. Tuner farge '{self.color}'. "
            f"'s' lagrer til {self.output_file}, 'q'/ESC avslutter.")

    def _read_trackbars(self):
        """Hent gjeldende trackbar-posisjoner som (lower, upper) HSV-tupler."""
        vals = {n: cv2.getTrackbarPos(n, WINDOW) for n in TRACKBARS}
        lower = np.array([vals['H low'], vals['S low'], vals['V low']],
                         dtype=np.uint8)
        upper = np.array([vals['H high'], vals['S high'], vals['V high']],
                         dtype=np.uint8)
        return lower, upper

    def image_callback(self, msg):
        # cv_bridge-konvertering: moenster fra Lab4 camera_pipeline.
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # BGR->HSV + inRange: OpenCV "Changing Colorspaces"-tutorial.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower, upper = self._read_trackbars()
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Vis original | maske | maskert resultat side om side. Generert.
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = cv2.hconcat([frame, mask_bgr, result])
        cv2.imshow(WINDOW, combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            self._save(lower, upper)
        elif key == ord('q') or key == 27:  # 27 = ESC
            rclpy.shutdown()

    def _save(self, lower, upper):
        # Lagre som EN range under fargenavnet. For roed maa du kjoere
        # to ganger (eller redigere YAML for hand) - H wrapper rundt
        # 0/180, se testlogg 20260512-09. Generert.
        entry = [{
            'h': [int(lower[0]), int(upper[0])],
            's': [int(lower[1]), int(upper[1])],
            'v': [int(lower[2]), int(upper[2])],
        }]
        data = {}
        if os.path.exists(self.output_file):
            with open(self.output_file) as f:
                data = yaml.safe_load(f) or {}
        data[self.color] = entry
        with open(self.output_file, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        self.get_logger().info(
            f"Lagret '{self.color}' = {entry} til {self.output_file}")


def main(args=None):
    rclpy.init(args=args)
    node = HsvTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
