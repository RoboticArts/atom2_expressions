#!/usr/bin/env python3

import rospy
from atom2_emotions.srv import SetExpression, SetExpressionResponse
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


newRosCommandFlag = False
newRosCommand = ""

def callback(req):

    global newRosCommand
    global newRosCommandFlag
    newRosCommand = req.expression
    newRosCommandFlag = True
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", newRosCommand)
    
    res = SetExpressionResponse()
    res.success = True

    return res 

def getRosCommand():

    global newRosCommand
    global newRosCommandFlag
    newRosCommandFlag = False
    return newRosCommand

def isNewRosCommand():
    return newRosCommandFlag

##################### Shared functions and variables for the threads ######################

emotion = ""
state = ""
finished = False
newEmotionCommandFlag = False

def getCurrentEmotion():
    global state
    global finished
    return state, finished

def getCurrentState():
    global state
    return state

def updateEmotionState(emotion_state, emotion_finished):
    global state   
    global finished
    state = emotion_state
    finished = emotion_finished


def setEmotionCommand(emotion_command):
    global emotion
    global finished 
    global newEmotionCommandFlag
    newEmotionCommandFlag = True
    finished = False
    emotion = emotion_command

def getEmotionCommand():
    global newEmotionCommandFlag
    newEmotionCommandFlag = False
    global emotion
    return emotion

def isNewEmotionCommand():
    return newEmotionCommandFlag

############################# END OF SHARED FUNCTIONS AND VARIABLES #######################


################### THREAD 1: graphical interface for atom2 eyes ##########################

frame = 0

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
            #playEmotionBlocking(idle_mp4, label)
            playFakeEmotionBlocking(command, "idle")

        if command == "go_left":
            #playEmotionBlocking(go_left_mp4, label)
            playFakeEmotionBlocking(command, "idle_left")

        if command == "back_left":
            #playEmotionBlocking(back_left_mp4, label)
            playFakeEmotionBlocking(command, "idle")

        if command == "go_right":
            #playEmotionBlocking(go_right_mp4, label)
            playFakeEmotionBlocking(command, "idle_right")

        if command == "back_right":
            #playEmotionBlocking(back_right_mp4, label)
            playFakeEmotionBlocking(command, "idle")



def playEmotionBlocking(video, label):

    frame = 0

    for image in video.iter_data():
        frame +=1
        image_frame = Image.fromarray(image)
        frame_image = ImageTk.PhotoImage(image_frame)
        label.config(image=frame_image)
        label.image = frame_image


def playFakeEmotionBlocking(emotion_command, emotion_idle):

    #Update status
    updateEmotionState(emotion_command, False)

    #Play video in blocking mode
    for x in range(0, 10):
        print(emotion_command)
        rospy.sleep(0.1)

    #Update status
    updateEmotionState(emotion_command, True)

    global frame
    frame = 0

    #Wait new command
    while(True):

        playIdleEmotion(emotion_idle) #This function uses frame variable

        #if emotion_command != getEmotionCommand():
        if isNewEmotionCommand():
            return
#idle inicial
# revisar idle sin espera
# revisar cuando se manda dos veces seguidas el mismo comando
# averiguar como leer de los frames del video

def playIdleEmotion(emotion_idle):

    global frame
    end_frame = 20

    if frame <= end_frame:
        frame+=1
        print(emotion_idle)
        print(frame)
        rospy.sleep(0.5)
    else:
        frame = 0

###################################### END OF THREAD 1 #####################################


################# THREAD 2: backend code to control graphical interface ####################

def interface_control():

    try:
        runInterfaceControl()
    except rospy.ROSInterruptException:
        pass


def runInterfaceControl():

    global emotion

    setEmotionCommand("idle")
    waitFaceInterface()

    while not rospy.is_shutdown():

        #print ("I'm interface control")

        if isNewRosCommand():

            rosscomand = getRosCommand()
            state = getCurrentState()

            if state.find("go") != -1:
                print("hey")
                undo_state = state.replace("go","back")
                setEmotionCommand(undo_state)
                waitFaceInterface()

                setEmotionCommand(rosscomand)
                waitFaceInterface()

            else:
                print("sitio")
                setEmotionCommand(rosscomand)
                waitFaceInterface()

        # setEmotionCommand("go_left")
        # waitFaceInterface()
        # print("Espera A terminada")

        # setEmotionCommand("go_right")
        # waitFaceInterface()
        # print("Espera B terminada")

        # setEmotionCommand("back_right")
        # waitFaceInterface()
        # print("Espera C terminada")



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

    rospy.init_node('atom2_face_node', anonymous=True)
    rospy.Service('set_expression', SetExpression, callback)

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

  


