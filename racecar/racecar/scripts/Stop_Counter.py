import cv2
import numpy as np
import time

class Stop_Counter:
    def __init__(self):
        self.cnt = 2
        self.previous_time = time.time() + 5
        self.lower_yellow = (20, 100, 100)
        self.upper_yellow = (40, 255, 255)

    def check_stop_line(self, img):
        if time.time() < self.previous_time + 10:
            return False

        img = img[240:] #240
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, self.lower_yellow, self.upper_yellow)

        # IMSHOW FOR DEBUG
        # print(np.count_nonzero(img_mask))
        cv2.putText(img_mask, 'NONZERO %d'%np.count_nonzero(img_mask), (0,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.imshow('img_mask', img_mask)

        if np.count_nonzero(img_mask) > 2500:
            self.on_detected()
            return True

        return False

    def on_detected(self):
        print('STOP LINE DETECTED!!!')
        self.previous_time = time.time()
        self.cnt += 1

if __name__ == '__main__':
    stop_counter = Stop_Counter()

    cap = cv2.VideoCapture('video/original.avi')
    while cap.isOpened():
        ret, frame = cap.read()
        stop_counter.check_stop_line(frame)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break
