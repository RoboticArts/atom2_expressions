#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import tkinter as tk, threading
import imageio
from PIL import Image, ImageTk
import time

shared_variable = 1

video_path = "/home/robert/catkin_ws/src/atom2_emotions/src/animations/"


idle_mp4       = imageio.get_reader(video_path + "idle.mp4")
go_left_mp4    = imageio.get_reader(video_path + "go_left.mp4" )
back_left_mp4  = imageio.get_reader(video_path + "back_left.mp4" )
go_right_mp4   = imageio.get_reader(video_path + "go_right.mp4" )
back_right_mp4 = imageio.get_reader(video_path + "back_right.mp4" )


def callback(data):
    global shared_variable
    shared_variable = 3
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


##################### Shared functions and variables for the threads ######################

emotion = "idle"
state = "idle"
finished = True

def getCurrentEmotion():
    global state
    global finished
    return state, finished

def updateEmotionState(emotion_state, emotion_finished):
    global state   
    global finished
    state = emotion_state
    finished = emotion_finished


def setEmotionCommand(emotion_command):
    global emotion
    emotion = emotion_command

def getEmotionCommand():
    global emotion
    return emotion

############################# END OF SHARED FUNCTIONS AND VARIABLES #######################


################### THREAD 1: graphical interface for atom2 eyes ##########################

def interface_face(label):

    try:
        runInterfaceFace(label)
    except rospy.ROSInterruptException:
        pass


def runInterfaceFace(label):

    while not rospy.is_shutdown():

        #Update command from control_interface
        command = getEmotionCommand()

        #print ("I'm interface face")
        
        if command == "idle":
            #print("idle")
            #playEmotionBlocking(idle_mp4, label)
            playFakeEmotionBlocking(command)

        if command == "go_left":
            #print("go_left")
            #playEmotionBlocking(go_left_mp4, label)
            playFakeEmotionBlocking(command)

        if command == "back_left":
            #print("back_left")
            #playEmotionBlocking(back_left_mp4, label)
            playFakeEmotionBlocking(command)

        if command == "idle_left":
            #print("idle_left")
            #playEmotionBlocking(idle_left_mp4, label)
            playFakeEmotionBlocking(command)

        if command == "go_right":
            #print("go_right")
            #playEmotionBlocking(go_right_mp4, label)
            playFakeEmotionBlocking(command)

        if command == "back_right":
            #print("back_right")
            #playEmotionBlocking(back_right_mp4, label)
            playFakeEmotionBlocking(command)

        if command == "idle_right":
            #print("idle_right")
            #playEmotionBlocking(idle_right_mp4, label)
            playFakeEmotionBlocking(command)


def playEmotionBlocking(video, label):

    frame = 0

    for image in video.iter_data():
        frame +=1
        image_frame = Image.fromarray(image)
        frame_image = ImageTk.PhotoImage(image_frame)
        label.config(image=frame_image)
        label.image = frame_image


def playFakeEmotionBlocking(emotion_command):

    #Update status
    updateEmotionState(emotion_command, False)

    #Play video in blocking mode
    for x in range(0, 10):
        print(emotion_command)
        rospy.sleep(0.1)

    #Update status
    updateEmotionState(emotion_command, True)

    #Wait new command
    while(True):
        if emotion_command != getEmotionCommand():
            return
    
    #Hold eyes. It changes go_emotion or back_emotion to idle_emotion
    #idle_command = emotion_command.replace("go", "idle")
    #idle_command = emotion_command.replace("back", "idle")
    #setEmotionCommand(idle_command)


# def playFakeEmotionPolling(emotion_command):

#     #Update the state
#     updateEmotionState(emotion_command)
    
#     #Infinite loop until new command
#     while(True):

#         #Play video in polling mode
#         for x in range(0, 10):

#             print(emotion_command)
#             rospy.sleep(0.1)

#             if(pollingNewCommands())
#                 return




###################################### END OF THREAD 1 #####################################




################# THREAD 2: backend code to control graphical interface ####################

def interface_control():

    try:
        runInterfaceControl()
    except rospy.ROSInterruptException:
        pass


def runInterfaceControl():

    global emotion

    while not rospy.is_shutdown():

        print ("I'm interface control")

        current_state, current_finished = getCurrentEmotion()

        if current_state.find("go") and current_finished:

            while(True):
                command = current_state.replace("go", "idle")
                waitFaceInterface()
                setEmotionCommand(command)  
                if checkCommandROS():
                    break

        if current_state.find("back") and current_finished:
            
            while(True):
                command = "idle"
                waitFaceInterface()
                setEmotionCommand(command) 
                if checkCommandROS():
                    break

        if current_state.find("idle") and current_finished:
            
            while(True):
                command = "idle"
                waitFaceInterface()
                setEmotionCommand(command) 
                if checkCommandROS():
                    break

        
        if checkCommandROS:
            #get ROS emotion command
            # First set idle emotion using go or back emotion
            # setEmotionCommand (ROS emotion command)


        waitFaceInterface()
        setEmotionCommand("go_left")

        waitFaceInterface()
        setEmotionCommand("go_right")

        waitFaceInterface()
        setEmotionCommand("back_right")

        waitFaceInterface()
        setEmotionCommand("go_left")

        waitFaceInterface()
        setEmotionCommand("back_left")

        waitFaceInterface()
        setEmotionCommand("idle")


def waitFaceInterface():
    
    # Wait until the emotion has finished
    while(True):
        state, finished = getCurrentEmotion()
        if finished:
            return





###################################### END OF THREAD 2 #####################################


#  global shared_variable

#  for x in range(0, 10):
#     frame = 0

#     if x <= 4:

#         for image in idle.iter_data():
#             frame += 1                                    #counter to save new frame number
#             image_frame = Image.fromarray(image)               #if you need the frame you can save each frame to hd
#             frame_image = ImageTk.PhotoImage(image_frame)
#             label.config(image=frame_image)
#             label.image = frame_image

#     if x == 5:
        
#         for image in go_left.iter_data():
#             frame += 1                                    #counter to save new frame number
#             image_frame = Image.fromarray(image)               #if you need the frame you can save each frame to hd
#             frame_image = ImageTk.PhotoImage(image_frame)
#             label.config(image=frame_image)
#             label.image = frame_image

#     if x == 6:
        
#         for image in back_left.iter_data():
#             frame += 1                                    #counter to save new frame number
#             image_frame = Image.fromarray(image)               #if you need the frame you can save each frame to hd
#             frame_image = ImageTk.PhotoImage(image_frame)
#             label.config(image=frame_image)
#             label.image = frame_image
    
#     if x >= 7:    

#         for image in idle.iter_data():
#             frame += 1                                    #counter to save new frame number
#             image_frame = Image.fromarray(image)               #if you need the frame you can save each frame to hd
#             frame_image = ImageTk.PhotoImage(image_frame)
#             label.config(image=frame_image)
#             label.image = frame_image

if __name__ == "__main__":

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)

    root = tk.Tk()
    my_label = tk.Label(root)
    my_label.pack()

    thread_face = threading.Thread(target=interface_face, args=(my_label,))
    thread_face.daemon = 1
    thread_face.start()

    thread_control = threading.Thread(target=interface_control)
    thread_control.daemon = 1
    thread_control.start()

    root.mainloop()

  


