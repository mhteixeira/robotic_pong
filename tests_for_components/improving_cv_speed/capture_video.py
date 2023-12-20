import cv2
import time
import glob
import re

video = cv2.VideoCapture(0)

frame_width = int(video.get(3))
frame_height = int(video.get(4))
   
size = (frame_width, frame_height)

file_list = glob.glob('./tests_for_components/improving_cv_speed/examples/*.avi')
existing_videos = [int(re.search('[0-9]{1,3}', name.split("\\")[-1])[0]) for name in file_list]
number_of_next_video = max(existing_videos) + 1
while True:
    try:
        input("Pressione qualquer tecla para começar a gravar...")
    except: 
        break
    
    result = cv2.VideoWriter(f'./tests_for_components/improving_cv_speed/examples/example{number_of_next_video}.avi', 
                            cv2.VideoWriter_fourcc(*'MJPG'),
                            10, size)

    while(True):
        ret, frame = video.read()
        if ret == True: 
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

    result.release()
    number_of_next_video += 1

video.release()
    
# Closes all the frames
cv2.destroyAllWindows()