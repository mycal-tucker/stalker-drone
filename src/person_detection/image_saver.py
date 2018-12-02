# Object-oriented approach to grabbing images from the mambo drone and
# saving them to a file and (maybe) returning the image in memory.

from pyparrot.Minidrone import Mambo
from pyparrot.DroneVision import DroneVision
import cv2

class UserVision:  # FIXME: I don't like this setup.
    def __init__(self, vision):
        self.index = 0
        self.vision = vision

    def save_pictures(self, args):
        print("in save pictures on image %d " % self.index)

        img = self.vision.get_latest_valid_picture()

        if img is not None:
            filename = "test_image_%06d.png" % self.index
            cv2.imwrite(filename, img)
            self.index += 1
            # Save another copy of the same image to a hardcoded filename, which
            # we'll use as the latest image
            magic_filename = "latest_image.png"
            cv2.imwrite(magic_filename, img)
            #print(self.index)


class ImageSaver:
    def __init__(self, mambo):
        self.mambo = mambo
        print("Creating image saver.")

    # When called, opens a video connection, saves images, and returns the latest image in memory.
    def get_latest_image(self):
        print("Called get latest image.")
        # Note: it is necessary to create new objects in this method instead of creating instance variables
        # in the constructor because of weird video threading issues.
        mambo_vision = DroneVision(self.mambo, is_bebop=False, buffer_size=30)
        user_vision = UserVision(mambo_vision)
        mambo_vision.set_user_callback_function(user_vision.save_pictures, user_callback_args=None)

        success = mambo_vision.open_video()  # Open the video stream using ffmpeg for capturing and processing
        print("Success in opening vision is %s" % success)

        print("Vision successfully started!")
        # FIXME: investigate if these sleeps are needed.
        self.mambo.smart_sleep(5)
        frame = mambo_vision.get_latest_valid_picture()
        self.mambo.smart_sleep(5)

        print("Ending the sleep and vision")
        mambo_vision.close_video()

        return frame
