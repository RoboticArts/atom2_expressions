#!/usr/bin/env python3

# import imageio
# from tkinter import Tk, Label
# from PIL import ImageTk, Image
# from pathlib import Path

# #imageio.plugins.ffmpeg.download()

# video_name = "/home/robert/catkin_ws/src/atom2_eyes/src/animations/idle.mp4"
# video = imageio.get_reader(video_name)
# delay = int(1000 / video.get_meta_data()['fps'])

# def stream(label):
 
#   try:
#     image = video.get_next_data()
#   except:
#     video.close()
#     return
#   label.after(delay, lambda: stream(label))
#   frame_image = ImageTk.PhotoImage(Image.fromarray(image))
#   label.config(image=frame_image)
#   label.image = frame_image

# if __name__ == '__main__':
 
#   root = Tk()
#   my_label = Label(root)
#   my_label.pack()
#   my_label.after(delay, lambda: stream(my_label))
#   root.mainloop()


# import tkinter as tk, threading
# import imageio
# from PIL import Image, ImageTk

# video_name = "/home/robert/catkin_ws/src/atom2_eyes/src/animations/idle.mp4" #This is your video file path
# #video = imageio.get_reader(video_name)
# #num_frames=video.get_meta_data()['nframes']

# def stream(label):

#   for x in range(0, 5):

#     video = imageio.get_reader(video_name)

#     for image in video.iter_data():
#         frame_image = ImageTk.PhotoImage(Image.fromarray(image))
#         label.config(image=frame_image)
#         label.image = frame_image  


# if __name__ == "__main__":

#     root = tk.Tk()
#     my_label = tk.Label(root)
#     my_label.pack()
#     thread = threading.Thread(target=stream, args=(my_label,))
#     thread.daemon = 1
#     thread.start()
#     root.mainloop()


import rospy
from std_msgs.msg import String
import tkinter as tk, threading
import imageio
from PIL import Image, ImageTk

shared_variable = 1



video_name = "/home/robert/catkin_ws/src/atom2_eyes/src/animations/idle.mp4" #This is your video file path
video = imageio.get_reader(video_name)

video2_name = "/home/robert/catkin_ws/src/atom2_eyes/src/animations/go_left.mp4" 
video2 = imageio.get_reader(video2_name)

video3_name = "/home/robert/catkin_ws/src/atom2_eyes/src/animations/back_left.mp4" 
video3 = imageio.get_reader(video3_name)

video4_name = "/home/robert/catkin_ws/src/atom2_eyes/src/animations/idle.mp4" 
video4 = imageio.get_reader(video4_name)


def callback(data):
    global shared_variable
    shared_variable = 3
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def stream(label):

 global shared_variable

 for x in range(0, 10):
    frame = 0

    if x <= 4:

        for image in video.iter_data():
            frame += 1                                    #counter to save new frame number
            image_frame = Image.fromarray(image)               #if you need the frame you can save each frame to hd
            frame_image = ImageTk.PhotoImage(image_frame)
            label.config(image=frame_image)
            label.image = frame_image

    if x == 5:
        
        for image in video2.iter_data():
            frame += 1                                    #counter to save new frame number
            image_frame = Image.fromarray(image)               #if you need the frame you can save each frame to hd
            frame_image = ImageTk.PhotoImage(image_frame)
            label.config(image=frame_image)
            label.image = frame_image

    if x == 6:
        
        for image in video3.iter_data():
            frame += 1                                    #counter to save new frame number
            image_frame = Image.fromarray(image)               #if you need the frame you can save each frame to hd
            frame_image = ImageTk.PhotoImage(image_frame)
            label.config(image=frame_image)
            label.image = frame_image
        while(True):
            print("lock")
            print(shared_variable)
    
    if x >= 7:    

        for image in video4.iter_data():
            frame += 1                                    #counter to save new frame number
            image_frame = Image.fromarray(image)               #if you need the frame you can save each frame to hd
            frame_image = ImageTk.PhotoImage(image_frame)
            label.config(image=frame_image)
            label.image = frame_image

if __name__ == "__main__":

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)

    root = tk.Tk()
    my_label = tk.Label(root)
    my_label.pack()
    thread = threading.Thread(target=stream, args=(my_label,))
    thread.daemon = 1
    thread.start()

    root.mainloop()


# import vlc
# from time import sleep

# Instance = vlc.Instance('--input-repeat=2', '--no-video-title-show', '--fullscreen', '--mouse-hide-timeout=0')
# player = Instance.media_player_new()
# Media = Instance.media_new('/home/robert/catkin_ws/src/atom2_eyes/src/animations/idle.mp4')
# Media.get_mrl()
# player.set_media(Media)
# player.play()

# print("aa")

# sleep(5) # Or however long you expect it to take to open vlc
# while player.is_playing():
#      sleep(1)

# print("aa")

# Instance = vlc.Instance('--input-repeat=2', '--no-video-title-show', '--fullscreen', '--mouse-hide-timeout=0')
# player = Instance.media_player_new()
# Media = Instance.media_new('/home/robert/catkin_ws/src/atom2_eyes/src/animations/go_left.mp4')
# Media.get_mrl()
# player.set_media(Media)
# player.play()

# while player.is_playing():
#      sleep(1)


# import vlc
# import time

# my_list = ['/home/robert/catkin_ws/src/atom2_eyes/src/animations/idle.mp4',
#  '/home/robert/catkin_ws/src/atom2_eyes/src/animations/go_left.mp4',
#  '/home/robert/catkin_ws/src/atom2_eyes/src/animations/back_left.mp4',
#  '/home/robert/catkin_ws/src/atom2_eyes/src/animations/go_right.mp4',
#  '/home/robert/catkin_ws/src/atom2_eyes/src/animations/back_right.mp4']
# instance = vlc.Instance()
# player = instance.media_player_new()
# playing = set([1,2,3,4])
# for i in my_list:
#     player.set_mrl(i)
#     player.play()
#     play=True
#     while play == True:
#         time.sleep(1)
#         play_state = player.get_state()
#         if play_state in playing:
#             continue
#         else:
#             play = False
