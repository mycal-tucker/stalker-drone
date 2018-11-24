from utils.bounding_box import BoundingBox
import cv2
import imutils
from imutils.object_detection import non_max_suppression
import numpy as np


# Object for encapsulating the logic to detect people in images. The key functionality is
# in the detect_person method, which takes in a path to a saved image on disk and returns
# a list of bounding boxes describing where there are people in the image.
class PersonDetector:
    # Initialize a few helper objects that will be used for detecting people.
    def __init__(self):
        # FIXME: why are we using HOG and SVMs instead of tensorflow?
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # Given a path to an image, returns a list of bounding boxes of the people in the image.
    # We use a list rather than a single BoundingBox in case there are multiple people in the
    # photo at the same time.
    def detect_person(self, image_filepath, visualize_bb=False):
        image = cv2.imread(image_filepath)
        # Resize image.
        image = imutils.resize(image, width=min(400, image.shape[1]))
        orig = image.copy()
        # Detect people in the image.
        rects, weights = self.hog.detectMultiScale(image, winStride=(4, 4), padding=(8, 8), scale=1.05)

        # Draw the original bounding boxes
        for (x, y, w, h) in rects:
            cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # apply non-maxima suppression to the bounding boxes using a
        # fairly large overlap threshold to try to maintain overlapping
        # boxes that are still people
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

        # Draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

        # show some information on the number of bounding boxes
        print("[INFO] {}: {} original boxes, {} after suppression".format(image_filepath, len(rects), len(pick)))

        # Show the output images
        if visualize_bb:
            cv2.imshow("Before NMS", orig)
            cv2.imshow("After NMS", image)
            cv2.waitKey(2000)

        # Convert from the rectangle format used in visualization to BoundingBoxes.
        bounding_boxes = []
        for (xA, yA, xB, yB) in pick:
            centroid = (np.mean((xA, xB)), np.mean((yA, yB)))
            width = np.abs(xA - xB)
            height = np.abs(yA - yB)
            bb = BoundingBox((width, height), centroid)
            bounding_boxes.append(bb)

        return bounding_boxes
