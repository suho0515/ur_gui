# opencv
import cv2

from PyQt5.QtCore import QThread

class CAM(QThread):
    def __init__(self, parent=None):
        super(CAM,self).__init__()
        self.main = parent
        self.isRun = False

        print("CAM Class is Initializing")

        # Initilaize Camera Sensor and Image
        self.cap = cv2.VideoCapture(-1)
        self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.img = None

    def save_image(self, path, num):
        #str_path = '/home/hri/catkin_ws/src/ur_gui/images/'

        str_name = path + "image_" + str(num) + ".jpg"

        cv2.imwrite(str_name, self.img)

    def run(self):
        while self.isRun:
            ret, self.img = self.cap.read()

            if not ret:
                    print("failed to grab frame")
                    continue

        self.cap.release()


