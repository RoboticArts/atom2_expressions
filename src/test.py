#!/usr/bin/env python3

import tkinter as tk, threading
import imageio
from PIL import Image, ImageTk
import time

video_name = "/home/robert/catkin_ws/src/atom2_emotions/src/animations/idle.mp4" 
video = imageio.get_reader(video_name)
video_aux = video

width  = int(1280/2)
height = int(720/2)
size = 100,100

def stream(label):

    frame = 0
    end_frame =  len(list(video.iter_data()))

    tic = 0
    toc = 0

    while(True):
        
        while(True): 

            frame+=1

            if frame == 1 or frame == 2 or frame == end_frame-1:
                 tic = time.time()

            image = video.get_next_data()

            img = Image.fromarray(image)
            img = img.resize((width, height), Image.ANTIALIAS)
            #img.thumbnail(size,Image.ANTIALIAS)

            if frame == 1 or frame == 2 or frame == end_frame-1:
                toc = time.time()
                print(toc-tic)

            frame_image = ImageTk.PhotoImage(img)
            #frame_image = ImageTk.PhotoImage(Image.fromarray(image))
            label.config(image=frame_image)
            label.image = frame_image

            if frame < end_frame-1:
                print ("Frame: ", frame)
            else:    
                print ("End frame: ", end_frame)
                frame = 0
                video.set_image_index(0)
                tic = time.time()
                image = video.get_next_data()
                toc = time.time()
                print("oket:", toc-tic)

                break

            
            
if __name__ == "__main__":

    root = tk.Tk()

    ws = root.winfo_screenwidth() # width of the screen
    hs = root.winfo_screenheight() # height of the screen
    x = (ws/2)
    y = -100 + ((hs/2))
    root.geometry('%dx%d+%d+%d' % (ws, hs, x, y))
    root.configure(bg='black')

    my_label = tk.Label(root)
    my_label.pack()
    my_label.config(fg="black", bg="black")
    my_label.place(x=ws/2, y=hs/2, anchor="center")
    thread = threading.Thread(target=stream, args=(my_label,))
    thread.daemon = 1
    thread.start()



    root.mainloop()