from imutils.video import VideoStream
from six.moves import urllib
import cv2
import time
import numpy as np

url = 'http://10.1.24.167:8080//shot.jpg'

vs = VideoStream(src=0).start()
time.sleep(1.0)
imgResponse = urllib.request.urlopen(url)

imgNp = np.array(bytearray(imgResponse.read()),dtype=np.uint8)
frame = cv2.imdecode(imgNp,-1)

frame_height, frame_width, _ = frame.shape
size = (frame_width, frame_height)
   
# Below VideoWriter object will create
# a frame of above defined The output 
# is stored in 'filename.avi' file.
result = cv2.VideoWriter('filename.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)
    
while(True):
    imgResponse = urllib.request.urlopen(url)

    imgNp = np.array(bytearray(imgResponse.read()),dtype=np.uint8)
    frame = cv2.imdecode(imgNp,-1)
  
  
    # Write the frame into the
    # file 'filename.avi'
    result.write(frame)

    # Display the frame
    # saved in the file
    cv2.imshow('Frame', frame)

    # Press S on keyboard 
    # to stop the process
    if cv2.waitKey(1) & 0xFF == ord('s'):
        break
  
result.release()
cv2.destroyAllWindows()
   
print("The video was successfully saved")