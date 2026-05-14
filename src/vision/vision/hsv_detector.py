"""
hsv_detector - detekterer fargede kuber i kamerabildet via HSV-terskling.

Abonnerer paa kamera-stroemmen, bygger en HSV-maske per farge fra
hsv_thresholds.yaml, finner konturer og publiserer hver kube som en
Detection2D med bounding box og sentroide (i piksler). Publiserer ogsaa
et annotert debug-bilde for visuell verifisering i rqt_image_view.

Denne noden gir piksel-posisjon (u,v). Metrisk posisjon (x,y,z i
robot-base-frame) kommer i fase 2 naar intrinsisk kalibrering +
kamera->base-transform er paa plass.

Opphav per blokk:
  - rclpy-node + cv_bridge image-subscriber/-publisher: moenster fra
    Lab4 camera_pipeline (AIS2105).
  - BGR->HSV + inRange per fargemaske: OpenCV "Changing Colorspaces"-
    tutorial (https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html).
  - findContours + boundingRect + moments for sentroide: OpenCV
    "Contour Features"-tutorial
    (https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html).
  - Morfologisk opening/closing for stoeyfjerning, multi-range-maske
    (OR av flere HSV-bokser for roed), Detection2DArray-mapping og
    bilde-annoteringen: generert av Claude.
  - vision_msgs/Detection2DArray som meldingsformat: deklarert i
    package.xml fra prosjektstart; felt-API verifisert mot
    ros-perception/vision_msgs (ros2-branch).

Bruk:
    ros2 run vision hsv_detector --ros-args \\
        -p thresholds_file:=src/vision/config/hsv_thresholds.yaml
"""
import os

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

# BGR-tegnefarger for annotering, per kubefarge. Generert.
DRAW_COLOR = {
    'red': (0, 0, 255),
    'yellow': (0, 255, 255),
    'blue': (255, 0, 0),
}
DEFAULT_DRAW = (0, 255, 0)


class HsvDetector(Node):
    """Detekterer fargede kuber via HSV-terskling og konturanalyse."""

    def __init__(self):
        super().__init__('hsv_detector')

        # Parametere - alt som kan variere mellom lysforhold/oppsett er
        # eksponert, ikke hardkodet (ROS2-kategorien "konfigurerbart").
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('thresholds_file', 'hsv_thresholds.yaml')
        self.declare_parameter('min_area', 500)
        self.declare_parameter('blur_kernel', 5)
        self.declare_parameter('detections_topic', '/vision/detections')
        self.declare_parameter('annotated_topic', '/vision/image_annotated')

        image_topic = self.get_parameter('image_topic').value
        thresholds_file = self.get_parameter('thresholds_file').value
        self.min_area = self.get_parameter('min_area').value
        self.blur_kernel = self.get_parameter('blur_kernel').value

        self.thresholds = self._load_thresholds(thresholds_file)

        # cv_bridge-subscriber/-publisher: moenster fra Lab4 camera_pipeline.
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            self.get_parameter('detections_topic').value, 10)
        self.annotated_pub = self.create_publisher(
            Image, self.get_parameter('annotated_topic').value, 10)

        self.get_logger().info(
            f"hsv_detector i gang. Farger: {list(self.thresholds)}. "
            f"min_area={self.min_area} px^2.")

    def _load_thresholds(self, path):
        # Leser hsv_thresholds.yaml: {farge: [{h,s,v}, ...]}. Generert.
        if not os.path.exists(path):
            self.get_logger().error(
                f"Terskelfil ikke funnet: {path}. Kjoer hsv_tuner foerst, "
                f"eller send -p thresholds_file:=<sti>.")
            return {}
        with open(path) as f:
            return yaml.safe_load(f) or {}

    def _color_mask(self, hsv, ranges):
        # OR sammen en maske per HSV-boks - lar roed dekke begge ender
        # av hue-aksen (H wrapper rundt 0/180). inRange fra OpenCV
        # colorspaces-tutorial; multi-range-OR og morfologi generert.
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for r in ranges:
            lower = np.array([r['h'][0], r['s'][0], r['v'][0]], dtype=np.uint8)
            upper = np.array([r['h'][1], r['s'][1], r['v'][1]], dtype=np.uint8)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        # Morfologisk opening (fjern saltkorn) + closing (tett smaahull).
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def image_callback(self, msg):
        # cv_bridge-konvertering: moenster fra Lab4 camera_pipeline.
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Lett Gaussisk blur foer terskling demper sensorstoey. Generert.
        k = self.blur_kernel
        if k >= 3 and k % 2 == 1:
            frame_blur = cv2.GaussianBlur(frame, (k, k), 0)
        else:
            frame_blur = frame
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

        annotated = frame.copy()
        det_array = Detection2DArray()
        det_array.header = msg.header

        for color, ranges in self.thresholds.items():
            mask = self._color_mask(hsv, ranges)
            # findContours + contourArea + boundingRect + moments:
            # OpenCV "Contour Features"-tutorial.
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                m = cv2.moments(cnt)
                if m['m00'] == 0:
                    continue
                cx = m['m10'] / m['m00']
                cy = m['m01'] / m['m00']

                det_array.detections.append(
                    self._make_detection(msg.header, color, cx, cy, w, h))
                self._annotate(annotated, color, x, y, w, h, cx, cy)

        self.detection_pub.publish(det_array)
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        if det_array.detections:
            found = ', '.join(
                f"{d.id}@({d.bbox.center.position.x:.0f},"
                f"{d.bbox.center.position.y:.0f})"
                for d in det_array.detections)
            self.get_logger().info(
                f"Detektert: {found}", throttle_duration_sec=1.0)

    def _make_detection(self, header, color, cx, cy, w, h):
        # Mapping til vision_msgs/Detection2D. Felt-API verifisert mot
        # ros-perception/vision_msgs (ros2-branch). Generert.
        det = Detection2D()
        det.header = header
        det.id = color
        det.bbox.center.position.x = float(cx)
        det.bbox.center.position.y = float(cy)
        det.bbox.center.theta = 0.0
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = color
        # HSV gir ingen ekte sannsynlighet. 1.0 som plassholder - kan
        # byttes til en areal-/fyllgrad-proxy senere hvis vi vil rangere.
        hyp.hypothesis.score = 1.0
        det.results.append(hyp)
        return det

    def _annotate(self, img, color, x, y, w, h, cx, cy):
        # Tegn bounding box + sentroide + fargenavn. Generert.
        draw = DRAW_COLOR.get(color, DEFAULT_DRAW)
        cv2.rectangle(img, (x, y), (x + w, y + h), draw, 2)
        cv2.circle(img, (int(cx), int(cy)), 4, draw, -1)
        cv2.putText(img, color, (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, draw, 2)


def main(args=None):
    rclpy.init(args=args)
    node = HsvDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
