# #!/usr/bin/env python3

# # NOTE: this example requires PyAudio because it uses the Microphone class

# import speech_recognition as sr


# # obtain audio from the microphone
# r = sr.Recognizer()


# import speech_recognition as sr
# for index, name in enumerate(sr.Microphone.list_microphone_names()):
#     print("Microphone with name \"{1}\" found for `Microphone(device_index={0})`".format(index, name))

# with sr.Microphone(sample_rate=44100) as source:
#     #r.adjust_for_ambient_noise(source)  # here
#     print("Say something!")
#     audio = r.listen(source,timeout=5, phrase_time_limit=3)


# # recognize speech using Sphinx
# #try:
# #    print("Sphinx thinks you said " + r.recognize_sphinx(audio))
# #except sr.UnknownValueError:
# #    print("Sphinx could not understand audio")
# #except sr.RequestError as e:
# #    print("Sphinx error; {0}".format(e))


# # recognize speech using Google Speech Recognition
# try:
#     # for testing purposes, we're just using the default API key
#     # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
#     # instead of `r.recognize_google(audio)`
#     print("Google Speech Recognition thinks you said " + r.recognize_google(audio, language="es-ES"))
# except sr.UnknownValueError:
#     print("Google Speech Recognition could not understand audio")
# except sr.RequestError as e:
#     print("Could not request results from Google Speech Recognition service; {0}".format(e))




# # get audio from the microphone                                                                       

# import speech_recognition as sr

# r = sr.Recognizer()   

# keyword = "hello"   

# with sr.Microphone() as source:                                                                       
#     print("Speak:")                                                                   
#     audio = r.listen(source)   

# try:
#     if (r.recognize_google(audio) == keyword) :
#         print("hey!")
# except sr.UnknownValueError:
#     print("Could not understand audio")
# except sr.RequestError as e:
#     print("Could not request results; {0}".format(e))


import speech_recognition as sr

r = sr.Recognizer()

keyWord = 'atom'

with sr.Microphone() as source:
    print('Please start speaking..\n')
    r.adjust_for_ambient_noise(source)
    while True: 
        audio = r.listen(source)
        try:
            text = r.recognize_google(audio, language="es-ES")
            if keyWord.lower() in text.lower():
                print('Keyword detected in the speech.')
        except Exception as e:
            print('Please speak again.')