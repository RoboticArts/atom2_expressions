#!/usr/bin/env python3

"""
---------------------------------------------------------------------
Robotic Arts Industries
All Rights Reserved 2017-2020
---------------------------------------------------------------------
Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
https://creativecommons.org/licenses/by-nc-sa/4.0/
---------------------------------------------------------------------
File: atom2_face.py
Description:
This file creates a graphical interface to generate the eyes of the atom2 robot. 
Robot expressions are controlled from ROS through the set_expresion service.This
program controls expressions but does not apply logic. For full functionality the
following programs should be run:

            -   atom2_hearing: manage voice recognition
            -   atom2_vision: manage machine vision
            -   atom2_speaking: handles speech
            -   atom2_emotions_control: Apply logic based on the information received
                                        by vision and hearing. Shows emotions by 
                                        speech and eyes

----------------------------------------------------------------------
		Version: 0.0.1						| Last Modification: 19/04/2020
		Author:  Robert Vasquez Zavaleta
		Contact: roboticarts1@gmail.com
"""

import rospy
import sys
from atom2_expressions.srv import SetExpression, SetExpressionResponse
from std_msgs.msg import String
import tkinter as tk, threading
import imageio
from PIL import Image, ImageTk
import time
import os
import signal

# --------------------------------- Load animations ---------------------------------- #

video_list = {}
video_path = os.path.dirname(os.path.abspath(__file__)) + "/animations/"

video_list_name = [   
                    'idle',           
                    'go_left', 'back_left', 'idle_left',
                    'go_right', 'back_right', 'idle_right',
                    'go_left', 'back_left', 'idle_left',
                    'go_attentive', 'back_attentive', 'idle_attentive',
                    'go_laugh', 'back_laugh', 'idle_laugh',
                    'go_angry','back_angry', 'idle_angry',
                    'go_sad', 'back_sad', 'idle_sad',
                    'go_confused', 'back_confused', 'idle_confused']
              
for video_name in video_list_name:

    try:
        video_mp4 = imageio.get_reader(video_path + video_name + ".gif" ) 
        video_list.update( {video_name : video_mp4 } )

    except FileNotFoundError:
        print ("Error: Wrong path, cannot find " + video_name + ".gif")
        sys.exit(1)


# --------------------- Functions and variables for ROS logic  ----------------------- #

newRosCommandFlag = False
newRosCommand = ""
last_expression = "IDLE"

#Define the expressions accepted through rosservice
expressions_list = ['IDLE','LOOK_LEFT', 'LOOK_RIGHT', 'HAPPY', 'ANGRY', 'SAD','ATTENTIVE', 'CONFUSED']


def callbackRos(req):

    global newRosCommand
    global newRosCommandFlag
    global expressions_list
    global last_expression
    res = SetExpressionResponse()

    expression = req.expression

    if expression != last_expression:

        if expression in expressions_list:

            newRosCommandFlag = True

            switcher = {
            'IDLE':       "idle",
            'LOOK_LEFT':  "go_left",
            'LOOK_RIGHT': "go_right",
            'HAPPY':      "go_laugh",
            'ANGRY':      "go_angry",
            'SAD':        "go_sad",
            'ATTENTIVE':  "go_attentive",
            'CONFUSED':   "go_confused"
            }

            newRosCommand = switcher.get(expression)
            rospy.loginfo("Expression %s set successfully", expression) 
            res.success = True
            res.message = "Expression " + str(expression) + " set successfully"

            last_expression = expression

        else:
            rospy.loginfo("Expression %s does not exist", expression)
            res.success = False
            res.message = "Expression " + str(expression) + " does not exist"

    else:
            rospy.loginfo("Expression %s is already running", expression)
            res.success = False
            res.message = "Expression " + str(expression) + " is already running"

    return res 

def getRosCommand():

    global newRosCommand
    global newRosCommandFlag
    newRosCommandFlag = False
    return newRosCommand

def isNewRosCommand():
    global newRosCommandFlag
    return newRosCommandFlag


# ------------------ Shared functions and variables for the threads  -------------------- #

emotion = ""
state = ""
finished = False
newEmotionCommandFlag = False

# Functions for internal states
def getCurrentState():
    global state
    return state

def updateCurrentState(currentState):
    global state
    state = currentState

def stateFinished():
    global finished
    return finished

def isStateFinished(isFinished):
    global finished
    finished = isFinished

# Functions for communication between threads
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

# ------------------------ End of shared functions and variables  ----------------------- #


# --------------------- THREAD 1: graphical interface for atom2 eyes ---------------------#

def interface_face(label):

    try:
        runInterfaceFace(label)
    except rospy.ROSInterruptException:
        pass


def runInterfaceFace(label):

    while not rospy.is_shutdown():

        #Update command from control_interface thread
        command = getEmotionCommand()

        fromTo = {
            'idle':            ['idle','idle'],
            'go_left':         ['go_left','idle_left'],
            'back_left':       ['back_left','idle'],
            'go_right':        ['go_right','idle_right'],
            'back_right':      ['back_right','idle'],
            'go_attentive':    ['go_attentive','idle_attentive'],
            'back_attentive':  ['back_attentive','idle'],
            'go_laugh':        ['go_laugh','idle_laugh'],
            'back_laugh':      ['back_laugh','idle'],
            'go_angry':        ['go_angry','idle_angry'],
            'back_angry':      ['back_angry','idle'],
            'go_sad':          ['go_sad','idle_sad'],
            'back_sad':        ['back_sad','idle'],
            'go_confused':     ['go_confused','idle_confused'],
            'back_confused':   ['back_confused','idle']
        }

        fromState, toState = fromTo.get(command, ['idle','idle'])

        playEmotion(label, fromState, toState)


def playEmotion(label, video_name, video_idle_name):

    video = video_list[video_name]
    video_idle = video_list [video_idle_name]

    #Update state to communicate with control_interface thread
    updateCurrentState(video_name)
    
    isStateFinished(False)

    if video_name != "idle":
        playVideoBlocking(label, video)

    isStateFinished(True)

    #Wait new command, hold the idle position
    playVideoPolling(label, video_idle) 



def playVideoBlocking(label,video):

    for image in video.iter_data():
        frame_image = ImageTk.PhotoImage(Image.fromarray(image))
        label.config(image=frame_image)
        label.image = frame_image


def playVideoPolling(label, video): 

    while(True):

        for image in video.iter_data():
            frame_image = ImageTk.PhotoImage(Image.fromarray(image))
            label.config(image=frame_image)
            label.image = frame_image

            if isNewEmotionCommand():
                video.set_image_index(0)
                return


# ------------------------------------ End of THREAD 1 -----------------------------------#


# ----------------THREAD 2: backend code to control graphical interface ------------------#

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

        if isNewRosCommand():

            roscomand = getRosCommand()
            state = getCurrentState()

            if state.find("go") != -1:

                undo_state = state.replace("go","back")
                setEmotionCommand(undo_state)
                waitFaceInterface()

                setEmotionCommand(roscomand)
                waitFaceInterface()

            else:
                setEmotionCommand(roscomand)
                waitFaceInterface()

        rospy.sleep(0.05)


def waitFaceInterface():
    
    # Wait until the internal state is finished
    while(True):

        if stateFinished():
            return

        rospy.sleep(0.05)


# ------------------------------------ End of THREAD 2 -----------------------------------#


# --------------------------------------- Main code --------------------------------------#

def sigint_handler(sig, frame):
    root.quit()
    root.update()


if __name__ == "__main__":

    # Tkinter instance
    root = tk.Tk()

    # Set signal to exit using terminal
    signal.signal(signal.SIGINT, sigint_handler)
    
    # Init ROS node and service
    rospy.init_node('atom2_face_node', anonymous=True)
    rospy.Service('set_expression', SetExpression, callbackRos)

    # Configure label for video streaming
    label = tk.Label(root)
    label.pack()
    label.config(fg="black", bg="black")
    label.place(relx=.5, rely=.5, anchor="center")

    # Thread one: graphical interface
    thread_face = threading.Thread(target=interface_face, args=(label,))
    thread_face.daemon = 1
    thread_face.start()

    # Thread two: control interface
    thread_control = threading.Thread(target=interface_control)
    thread_control.daemon = 1
    thread_control.start()

    # Load icon
    try:
        root.tk.call('wm', 'iconphoto', root._w, tk.PhotoImage(file=video_path+'icon.png'))
    except:
        print("Icon not found")

    # Fills the entire screen
    ws = root.winfo_screenwidth()
    hs = root.winfo_screenheight() 
    x = (ws/2)
    y = -100 + ((hs/2))
    root.geometry('%dx%d+%d+%d' % (ws, hs, x, y))
    root.configure(bg='black')
    
    # Launch the application
    root.mainloop()
