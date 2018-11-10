# Class for representing the information in a bounding box of a person.
# Contains width and height of the box, in pixels, and the centroid of
# the box in the image.
class BoundingBox:
    def __init__(self, dimensions, centroid):
        # Dimensions is a 2-element tuple of width, height
        self.dimensions = dimensions
        # Centroid is a 2-element tuple of x, y of center of box in the raw image
        self.centroid = centroid

    def get_dimensions(self):
        return self.dimensions

    def get_centroid(self):
        return self.centroid

    def get_area(self):
        return self.dimensions[0] * self.dimensions[1]

    def __eq__(self, other):
        if not isinstance(other, BoundingBox):
            return False
        return other.dimensions == self.dimensions and other.centroid == self.centroid

    def __hash__(self):
        return 17 * self.dimensions.hash() + self.centroid.hash()
