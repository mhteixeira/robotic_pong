# Python program to save a 
# video using OpenCV
   
import cv2
import time
import glob
import re

time.sleep(2)
# Create an object to read 
# from camera
video = cv2.VideoCapture(1)

# We need to check if camera
# is opened previously or not
if (video.isOpened() == False): 
    print("Error reading video file")
  
# We need to set resolutions.
# so, convert them from float to integer.
frame_width = int(video.get(3))
frame_height = int(video.get(4))
   
size = (frame_width, frame_height)


# Using glob and regex, I find the number of the current video to be saved
file_list = glob.glob('./refactoring_cv/examples/*.avi')
existing_videos = [int(re.search('[0-9]{1,3}', name.split("\\")[-1])[0]) for name in file_list]
number_of_next_video = max(existing_videos) + 1

# Below VideoWriter object will create
# a frame of above defined The output 
# is stored in 'filename.avi' file.
result = cv2.VideoWriter(f'./refactoring_cv/examples/example{number_of_next_video}.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)
    
while(True):
    ret, frame = video.read()
  
    if ret == True: 
  
        # Write the frame into the
        # file 'filename.avi'
        result.write(frame)
  
        # Display the frame
        # saved in the file
        cv2.imshow('Frame', frame)
  
        # Press S on keyboard 
        # to stop the process
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
  
    # Break the loop
    else:
        break
  
# When everything done, release 
# the video capture and video 
# write objects
video.release()
result.release()
    
# Closes all the frames
cv2.destroyAllWindows()
   
print("The video was successfully saved")