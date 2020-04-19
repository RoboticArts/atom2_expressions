#!/usr/bin/env python3

import rospy
import sys
from atom2_emotions.srv import SetExpression, SetExpressionResponse
from std_msgs.msg import String
import tkinter as tk, threading
import imageio
from PIL import Image, ImageTk
import time
import os
import errno
import signal
from itertools import count

################### START: Load gifs ##########################


video_path = os.path.dirname(os.path.abspath(__file__)) + "/animations/"

try:
    idle_mp4 = imageio.get_reader(video_path + "idle.gif")
    go_left_mp4 = imageio.get_reader(video_path + "go_left.gif" )
    back_left_mp4 = imageio.get_reader(video_path + "back_left.gif" )
    idle_left_mp4 = imageio.get_reader(video_path + "idle_left.gif" )
    go_right_mp4 = imageio.get_reader(video_path + "go_right.gif" )
    back_right_mp4 = imageio.get_reader(video_path + "back_right.gif" )
    idle_right_mp4  = imageio.get_reader(video_path + "idle_right.gif" )
    go_attention_mp4 = imageio.get_reader(video_path + "go_attention.gif" )
    back_attention_mp4 = imageio.get_reader(video_path + "back_attention.gif" )
    idle_attention_mp4 = imageio.get_reader(video_path + "idle_attention.gif" )
    go_laugh_mp4 = imageio.get_reader(video_path + "go_laugh.gif" )
    back_laugh_mp4 = imageio.get_reader(video_path + "back_laugh.gif" )
    idle_laugh_mp4 = imageio.get_reader(video_path + "idle_laugh.gif" )
    go_angry_mp4 = imageio.get_reader(video_path + "go_angry.gif" )
    back_angry_mp4 = imageio.get_reader(video_path + "back_angry.gif" )
    idle_angry_mp4 = imageio.get_reader(video_path + "idle_angry.gif" )
    go_sad_mp4 = imageio.get_reader(video_path + "go_sad.gif" )
    back_sad_mp4 = imageio.get_reader(video_path + "back_sad.gif" )
    idle_sad_mp4 = imageio.get_reader(video_path + "idle_sad.gif" )
    go_confused_mp4 = imageio.get_reader(video_path + "go_confused.gif" )
    back_confused_mp4 = imageio.get_reader(video_path + "back_confused.gif" )
    idle_confused_mp4 = imageio.get_reader(video_path + "idle_confused.gif" )

except FileNotFoundError:
    print ("Error: Wrong path, cannot find animations")
    sys.exit(1)

################### ROS: Functions and variables for ROS logic ##########################

newRosCommandFlag = False
newRosCommand = ""
last_expression = "IDLE"
expressions_list = ['IDLE','LOOK_LEFT', 'LOOK_RIGHT', 'HAPPY', 'ANGRY', 'SAD','ATTENTIVE', 'CONFUSED']


def callback(req):

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
            'ATTENTIVE':  "go_attention",
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


##################### Shared functions and variables for the threads ######################

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

############################# END OF SHARED FUNCTIONS AND VARIABLES #######################


################### THREAD 1: graphical interface for atom2 eyes ##########################


def interface_face(label):

    try:
        runInterfaceFace(label)
    except rospy.ROSInterruptException:
        pass


def runInterfaceFace(label):

    while not rospy.is_shutdown():

        #Update command from control_interface thread
        command = getEmotionCommand()

        if command == "idle":
            playEmotion(label, command, "idle", idle_mp4, idle_mp4)
 
        if command == "go_left":
            playEmotion(label, command, "idle_left", go_left_mp4, idle_left_mp4)

        if command == "back_left":
            playEmotion(label, command, "idle", back_left_mp4, idle_mp4)

        if command == "go_right":
            playEmotion(label, command, "idle_right", go_right_mp4, idle_right_mp4)

        if command == "back_right":
            playEmotion(label, command, "idle", back_right_mp4, idle_mp4)

        if command == "go_attention":
            playEmotion(label, command, "idle_attention", go_attention_mp4, idle_attention_mp4)

        if command == "back_attention":
            playEmotion(label, command, "idle", back_attention_mp4, idle_mp4)

        if command == "go_laugh":
            playEmotion(label, command, "idle_laugh", go_laugh_mp4, idle_laugh_mp4)

        if command == "back_laugh":
            playEmotion(label, command, "idle", back_laugh_mp4, idle_mp4)

        if command == "go_angry":
            playEmotion(label, command, "idle_angry", go_angry_mp4, idle_angry_mp4)

        if command == "back_angry":
            playEmotion(label, command, "idle", back_angry_mp4, idle_mp4)

        if command == "go_sad":
            playEmotion(label, command, "idle_sad", go_sad_mp4, idle_sad_mp4)

        if command == "back_sad":
            playEmotion(label, command, "idle", back_sad_mp4, idle_mp4)
        
        if command == "go_confused":
            playEmotion(label, command, "idle_confused", go_confused_mp4, idle_confused_mp4)

        if command == "back_confused":
            playEmotion(label, command, "idle", back_confused_mp4, idle_mp4)


def playEmotion(label, emotion_command, emotion_idle_command, video, video_idle):

    #Update state to communicate with control_interface thread
    updateCurrentState(emotion_command)
    
    isStateFinished(False)

    playVideoBlocking(label, video)

    isStateFinished(True)

    #Wait new command, hold the idle position
    while(True):

        playVideoBackground(label, emotion_idle_command, video_idle) 

        if isNewEmotionCommand():
            return


def playVideoBlocking(label,video):

    for image in video.iter_data():
        frame_image = ImageTk.PhotoImage(Image.fromarray(image))
        label.config(image=frame_image)
        label.image = frame_image
        

frame = 0
end_frame = 0

def playVideoBackground(label,emotion_idle_command, video_idle):

    global frame
    global end_frame

    if frame == 0:
        video_idle.set_image_index(0)

    frame+=1
    end_frame = getEndFrame(emotion_idle_command)
    print(frame)
    image = video_idle.get_next_data() 
    frame_image = ImageTk.PhotoImage(Image.fromarray(image))
    label.config(image=frame_image)
    label.image = frame_image

    if frame >= end_frame:
        video_idle.set_image_index(0)
        frame = 0


def getEndFrame(video_name):

    switcher = {
            'idle': 80,
            'go_left': 9,
            'back_left': 9,
            'idle_left': 26,
            'go_right': 9,
            'back_right': 9,
            'idle_right': 26,
            'go_left': 9,
            'back_left': 9,
            'idle_left': 26,
            'go_attention': 10,
            'back_attention': 6,
            'idle_attention': 26,
            'go_laugh': 2,
            'back_laugh': 7,
            'idle_laugh': 33,
            'go_angry': 8,
            'back_angry': 8,
            'idle_angry': 26,
            'go_sad': 7,
            'back_sad': 11,
            'idle_sad': 26,
            'go_confused': 6,
            'back_confused': 6,
            'idle_confused': 26,
            }

    end_frame = switcher.get(video_name)

    return end_frame


def printNumberOfFrames():

    video_list = [
              idle_mp4,
              go_left_mp4, back_left_mp4, idle_left_mp4,
              go_right_mp4, back_right_mp4, idle_right_mp4,
              go_left_mp4, back_left_mp4, idle_left_mp4,
              go_attention_mp4, back_attention_mp4, idle_attention_mp4,
              go_laugh_mp4, back_laugh_mp4, idle_laugh_mp4,
              go_angry_mp4, back_angry_mp4, idle_angry_mp4,
              go_sad_mp4, back_sad_mp4, idle_sad_mp4,
              go_confused_mp4, back_confused_mp4, idle_confused_mp4]

    video_list_name = [   
              'idle',           
              'go_left', 'back_left', 'idle_left',
              'go_right', 'back_right', 'idle_right',
              'go_left', 'back_left', 'idle_left',
              'go_attention', 'back_attention', 'idle_attention',
              'go_laugh', 'back_laugh', 'idle_laugh',
              'go_angry','back_angry', 'idle_angry',
              'go_sad', 'back_sad', 'idle_sad',
              'go_confused', 'back_confused', 'idle_confused']
              
    video_name = 0

    for video in video_list:
        print ("'" + str(video_list_name[video_name]) + "': "+ str(len(list(video.iter_data()))) + ",")
        video_name+=1

    time.sleep(10)

# Añadir funcion de play especifica para idle
# Limpiar el código que sobre o organizarlo mejor


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




###################################### END OF THREAD 2 #####################################

root = tk.Tk()

def sigint_handler(sig, frame):
    root.quit()
    root.update()


if __name__ == "__main__":

    #printNumberOfFrames()

    signal.signal(signal.SIGINT, sigint_handler)
    rospy.init_node('atom2_face_node', anonymous=True)
    rospy.Service('set_expression', SetExpression, callback)

    my_label = tk.Label(root)
    my_label.pack()
    my_label.config(fg="black", bg="black")
    my_label.place(relx=.5, rely=.5, anchor="center")

    thread_face = threading.Thread(target=interface_face, args=(my_label,))
    thread_face.daemon = 1
    thread_face.start()

    thread_control = threading.Thread(target=interface_control)
    thread_control.daemon = 1
    thread_control.start()

    ws = root.winfo_screenwidth() # width of the screen
    hs = root.winfo_screenheight() # height of the screen
    x = (ws/2)
    y = -100 + ((hs/2))
    root.geometry('%dx%d+%d+%d' % (ws, hs, x, y))
    root.configure(bg='black')
    
    try:
        root.tk.call('wm', 'iconphoto', root._w, tk.PhotoImage(file=video_path+'icon_2.png'))
    except:
        print("Icon not found")

    root.mainloop()
  


