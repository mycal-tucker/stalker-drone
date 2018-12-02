import cv2
import unittest
from person_detection.tf_detector import TFDetector


class TestTFDetector(unittest.TestCase):
    # Before every test, reset the TFDetector object that will be tested.
    def setUp(self):
        self.tf_detector = TFDetector(model_filepath='../../src/person_detection/ssdlite_mobilenet_v2_coco_2018_05_09/frozen_inference_graph.pb',
                                      label_filepath='../../src/person_detection/mscoco_label_map.pbtxt')

    # Test that if there is one person in the image a non-None bounding box is returned.
    def test_one_person(self):
        # SETUP
        image_filepath = '../../data/drone_camera_test1.png'
        image = cv2.imread(image_filepath)

        # EXECUTE
        bb = self.tf_detector.detect_bounding_box(image)

        # VERIFY:
        print("YAY")
        assert bb is not None


if __name__ == '__main__':
    unittest.main()
