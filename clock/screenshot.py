import cv2
import os

start = 0.0               #seconds
end = 21                  #seconds
start_time = 175.0        #minutes
end_time = 198.0          #minutes
totl_time = end - start   #seconds
frame_step = 2
bounding_box = (850, 300, 1310 , 760)

short_video_name = "clock_9_2"
video_name =  short_video_name + ".mp4"
video_path = "C:/Users/mymri/repos/videos/" + video_name
output_folder = "output\\" + short_video_name 
out_string = "imagepath,minutes,video\n"

os.makedirs(output_folder, exist_ok=True)
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Error: failed to open the video.")
else:
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) 
    c = 0
    if end_time < start_time:
        delta = 720 - start_time + end_time
    else:
        delta = end_time - start_time

    for frame_number in range(2, total_frames, frame_step):  
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_number)  
        ret, frame = cap.read()
        if not ret:
            print(f"Failed to retrieve frame number {frame_number}.")
            break
        cropped_frame = frame[bounding_box[1]:bounding_box[3], bounding_box[0]:bounding_box[2]]
        
        t = frame_number / fps - start
        if t < 0 or t + start > end:
            continue
        time = start_time + delta * t / totl_time
        if(time > 720):
            time  -= 720

        short_name = f"frame_{c:04}.jpg"    
        filename = os.path.join(output_folder, short_name)
        c+=1
        cv2.imwrite(filename, cropped_frame)

        out_string += f"{short_name},{time:.2f},{video_name}\n"
        print(f"Frame saved: {filename}")
    
    with open(os.path.join(output_folder, "dataset_info.csv"), "w") as file:
        file.write(out_string)

    cap.release()