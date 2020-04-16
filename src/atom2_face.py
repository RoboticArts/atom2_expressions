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

end_frame_idle_mp4 =       len(list(idle_mp4.iter_data()))
end_frame_go_left_mp4 =    len(list(go_left_mp4.iter_data()))
end_frame_back_left_mp4 =  len(list(back_left_mp4.iter_data()))
end_frame_go_right_mp4 =   len(list(go_right_mp4.iter_data()))
end_frame_back_right_mp4 = len(list(back_right_mp4.iter_data()))

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
            'HAPPY':      "go_happy",
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
end_frame = 0

def interface_face(label):

    try:
        runInterfaceFace(label)
    except rospy.ROSInterruptException:
        pass


def runInterfaceFace(label):

    counter = 0

    while not rospy.is_shutdown():

        #Update command from control_interface
        command = getEmotionCommand()

        counter+=1

        # if counter == 1:
        #     command = "go_left"
        # if counter == 2:
        #     command = "idle"
        # if counter == 3:
        #     command = "back_right"

        #print ("I'm interface face")

        if command == "idle":
            dummy = 0
            playEmotionBlocking(label, command, "idle", idle_mp4, idle_mp4)
            #playFakeEmotionBlocking(command, "idle") 

        if command == "go_left":
            playEmotionBlocking(label, command, "go_left", go_left_mp4, go_left_mp4)
            #playFakeEmotionBlocking(command, "idle_left")

        if command == "back_left":
            playEmotionBlocking(label, command, "idle", back_left_mp4, back_left_mp4)
            #playFakeEmotionBlocking(command, "idle")

        if command == "go_right":
            playEmotionBlocking(label, command, "go_right", go_right_mp4, go_right_mp4)
            #playFakeEmotionBlocking(command, "idle_right")

        if command == "back_right":
            playEmotionBlocking(label, command, "idle", back_right_mp4, back_right_mp4)
            #playFakeEmotionBlocking(command, "idle")



def playEmotionBlocking(label, emotion_command, emotion_idle_command, video, video_idle):

    #Update status
    updateEmotionState(emotion_command, False)


    counter = 0
    for image in video.iter_data():
        frame_image = ImageTk.PhotoImage(Image.fromarray(image))
        label.config(image=frame_image)
        label.image = frame_image
        print(emotion_command)
        counter+=1
        print(counter)
        
    #Update status
    updateEmotionState(emotion_command, True)
    global frame
    global end_frame

    end_frame = getEndFrame(emotion_idle_command) 
    frame = 45
    video_idle.set_image_index(45)

    #Wait new command
    while(True):

        playIdleEmotion(label, emotion_idle_command, video_idle) #This function uses frame variable

        #if emotion_command != getEmotionCommand():
        if isNewEmotionCommand():
            return


def playIdleEmotion(label,emotion_idle_command, video_idle):
    
    global frame
    global end_frame

    frame+=1
    image = video_idle.get_next_data() 
    frame_image = ImageTk.PhotoImage(Image.fromarray(image))
    label.config(image=frame_image)
    label.image = frame_image
    print(emotion_idle_command)
    print(frame)

    if frame >= end_frame:
            video_idle.set_image_index(45)
            frame =45

def getEndFrame(video_name):

    global end_frame_idle_mp4 
    global end_frame_go_left_mp4
    global end_frame_back_left_mp4
    global end_frame_go_right_mp4
    global end_frame_back_right_mp4
  
    switcher = {
        'idle':       end_frame_idle_mp4,
        'go_left':    end_frame_go_left_mp4,
        'back_left':  end_frame_back_left_mp4,
        'idle_left':  end_frame_back_left_mp4,
        'go_right':   end_frame_go_right_mp4,
        'back_right': end_frame_back_right_mp4,
        'idle_right': end_frame_back_right_mp4,
        }

    end_frame = switcher.get(video_name)

    return end_frame


# def playFakeEmotionBlocking(emotion_command, emotion_idle):

#     #Update status
#     updateEmotionState(emotion_command, False)

#     if emotion_command != "idle":

#         #Play video in blocking mode
#         for x in range(0, 10):
#             print(emotion_command)
#             rospy.sleep(0.1)

#     #Update status
#     updateEmotionState(emotion_command, True)

#     global frame
#     frame = 0

#     #Wait new command
#     while(True):

#         playFakeIdleEmotion(emotion_idle) #This function uses frame variable

#         #if emotion_command != getEmotionCommand():
#         if isNewEmotionCommand():
#             return

# poner el index del video diferente de 0, restrasa entre un video y otro

# Revisar porque el video se para de una vez a otra
# Añadir funcion de play especifica para idle
# Comprobar que el nodo funciona la Jetson nano y la pantalla de 7''
# Renderizar videos con idle, reducir el tamaño de los videos
# Añadir los nuevos videos con las funciones y comprobar que funciona
# Limpiar el código que sobre o organizarlo mejor

# Esto de abajo no se si sigue pasando, revisar:
# # revisar porque se queda parado con el debug, no usar el idle del inicio y comprobar (last_expresion debe ser '')

# def playFakeIdleEmotion(emotion_idle):

#     global frame
#     end_frame = 20

#     if frame <= end_frame:
#         frame+=1
#         print(emotion_idle)
#         print(frame)
#         rospy.sleep(0.5)
#     else:
#         frame = 0



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

            roscomand = getRosCommand()
            state = getCurrentState()

            if state.find("go") != -1:

                print("hey")
                undo_state = state.replace("go","back")
                setEmotionCommand(undo_state)
                waitFaceInterface()

                setEmotionCommand(roscomand)
                waitFaceInterface()

            else:
                print("sitio")
                setEmotionCommand(roscomand)
                waitFaceInterface()

        rospy.sleep(0.05)

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

        rospy.sleep(0.05)




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

  


