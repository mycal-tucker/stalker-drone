import cv2
import unittest
from person_detection.tf_detector import TFDetector


class TestTFDetector(unittest.TestCase):
    # Before every test, reset the TFDetector object that will be tested.
    def setUp(self):
        self.tf_detector = TFDetector()

    # Test that if there is one person in the image a non-None bounding box is returned.
    def test_one_person(self):
        # SETUP
        image_filepath = '../../data/sertac.jpg'
        image = cv2.imread(image_filepath)

        # EXECUTE
        bb = self.tf_detector.detect_bounding_box(image)

        # VERIFY:
        print("YAY")
        assert bb is not None
        # TODO, once this works, add a better test.


if __name__ == '__main__':
    unittest.main()
