import cv2
import numpy as np


class CameraWrapper:
    """
    This is for the ZED Mini camera
    """
    def __init__(self):
        self.cap = None

    def set_cap(self, device_id):
        self.cap = cv2.VideoCapture(device_id)
        if self.cap.isOpened() == 0:
            return -1
        else:
            return 0

    def set_parameters(self):
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def read(self):
        retval, frame = self.cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        left_right_image = np.split(frame, 2, axis=1)
        return frame, left_right_image

if __name__ == '__main__':
    cam = CameraWrapper()
    cam.set_cap(2)
    cam.set_parameters()
    while True:
        frame, img = cam.read()
        cv2.imshow("frame", frame)
        cv2.imshow("right", img[0])
        cv2.imshow("left", img[1])
        if cv2.waitKey(30) >= 0:
            break
