import cv2
import threading
from pathlib import Path
from datetime import datetime

class MicroCamera(threading.Thread):
    def __init__(self, dev_id=4):
        threading.Thread.__init__(self)

        self.cap = cv2.VideoCapture(dev_id)
        self.stop_event = threading.Event()
        self.capture_event = threading.Event()
        self.capture_num = 0
        self.capture_dir = "./Data/captures"
        self.save_path = None
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

            if self.capturing_frame():
                if self.capture_num == 0:
                    # Generate a timestamp-based directory name
                    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                    self.save_path = Path(self.capture_dir, f"capture_{timestamp}")

                    if not self.save_path.exists():
                        self.save_path.mkdir(parents=True)
                        print(f"Directory created: {self.save_path}")
       
                self.capture_num += 1
                cv2.imwrite(Path(self.save_path, ("%04d.jpg" % self.capture_num)), frame)
                print("Captured image saved to %s" % Path(self.save_path, ("%04d.jpg" % self.capture_num)))
                self.capture_event.clear()

    # function using _stop function
    def stop(self):
        self.stop_event.set()

    def stopped(self):
        return self.stop_event.isSet()

    def capture_frame(self):
        self.capture_event.set()

    def capturing_frame(self):
        return self.capture_event.isSet()


if __name__ == "__main__":
    camera = MicroCamera()
    camera.start()
