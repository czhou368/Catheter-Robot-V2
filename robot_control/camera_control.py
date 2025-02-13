import cv2
import threading

class MicroCamera(threading.Thread):
    def __init__(self, dev_id=4):
        threading.Thread.__init__(self)

        self.cap = cv2.VideoCapture(dev_id)
        self.stop_event = threading.Event()

    def run(self):

        while True:
            ret, frame = self.cap.read()
            # print(ret)
            cv2.imshow("Camera Frame", frame)
            cv2.waitKey(1)

            if self.stopped():
                self.cap.release
                cv2.destroyAllWindows()
                return

    # function using _stop function
    def stop(self):
        self.stop_event.set()

    def stopped(self):
        return self.stop_event.isSet()


if __name__ == "__main__":
    camera = MicroCamera()
    camera.start()
