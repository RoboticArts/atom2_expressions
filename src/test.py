#!/usr/bin/env python3

import tkinter as tk, threading
import imageio
from PIL import Image, ImageTk

video_name = "/home/robert/catkin_ws/src/atom2_emotions/src/animations/idle.mp4" 
video = imageio.get_reader(video_name)

def stream(label):

    frame = 0
    end_frame =  len(list(video.iter_data()))

    while(True):
        
        while(True):
            
            frame+=1
            image = video.get_next_data()
            frame_image = ImageTk.PhotoImage(Image.fromarray(image))
            label.config(image=frame_image)
            label.image = frame_image

            if frame < end_frame:
                print ("Frame: ", frame)
            else:    
                print ("End frame: ", end_frame)
                frame = 0
                video.set_image_index(0)
                break
            


if __name__ == "__main__":

    root = tk.Tk()
    my_label = tk.Label(root)
    my_label.pack()
    thread = threading.Thread(target=stream, args=(my_label,))
    thread.daemon = 1
    thread.start()
    root.mainloop()