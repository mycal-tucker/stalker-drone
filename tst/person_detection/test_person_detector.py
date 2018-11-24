import unittest
from person_detection.person_detector import PersonDetector


class TestPersonDetector(unittest.TestCase):
    # Before every test, reset the PersonDetector object that will be tested.
    def setUp(self):
        self.person_detector = PersonDetector()

    # Test that if there is one person in the image, a list of size one is
    # returned.
    # FIXME: This test is failing because the vision code isn't detecting anyone.
    def test_one_person(self):
        # SETUP
        image_filepath = '../../data/sertac.jpg'

        # EXECUTE
        bbs = self.person_detector.detect_person(image_filepath, visualize_bb=True)

        # VERIFY:
        assert len(bbs) == 1


if __name__ == '__main__':
    unittest.main()
