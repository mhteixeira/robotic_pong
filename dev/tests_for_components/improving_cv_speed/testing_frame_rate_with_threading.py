import time
import unittest

import cv2

from VideoCaptureThreading import VideoCaptureThreading

class VideoCaptureTest(unittest.TestCase):
    def setUp(self):
        pass

    def _run(self, n_frames=500, width=1280, height=720, with_threading=False):
        if with_threading:
            cap = VideoCaptureThreading(0)
        else:
            cap = cv2.VideoCapture(0)
        if with_threading:
            cap.start()
        i = 0
        _, previous_frame = cap.read()
        t0 = time.time()
        while i < n_frames:
            _, frame = cap.read()
            cv2.imshow('Frame', frame)
            cv2.waitKey(1) & 0xFF
            if not (previous_frame == frame).all():
                i += 1
                previous_frame = frame.copy()
        filename = 'w_threading.txt' if with_threading else 'wo_threading.txt'
        with open(filename, 'a') as f:
            print({n_frames / (time.time() - t0)}, file=f)
        if with_threading:
            cap.stop()
        cv2.destroyAllWindows()

    def test_video_capture_threading(self):
        n_frames = 500
        for _ in range(20,):
            self._run(n_frames, 1280, 720, True)
            self._run(n_frames, 1280, 720, False)


if __name__ == '__main__':
    unittest.main()