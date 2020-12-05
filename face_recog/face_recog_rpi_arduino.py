import cv2
import numpy as np
import RPi.GPIO as GPIO
import collections

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(6, GPIO.IN)
GPIO.setup(13, GPIO.IN)
GPIO.setup(19, GPIO.IN)
GPIO.setup(26, GPIO.IN)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.setup(5, GPIO.IN)

input_1 = GPIO.input(6)
input_2 = GPIO.input(13)
input_3 = GPIO.input(19)
input_4 = GPIO.input(26)
input_5 = GPIO.input(5) 

# Create Local Binary Patterns Histograms for face recognization
recognizer = cv2.face.LBPHFaceRecognizer_create()
# Load the trained mode
recognizer.read('trainer/trainer.yml')
# Load prebuilt model for Frontal Face
cascadePath = "haarcascade_frontalface_default.xml"
# Create classifier from prebuilt model
faceCascade = cv2.CascadeClassifier(cascadePath)
# Set the font style
font = cv2.FONT_HERSHEY_SIMPLEX
# Initialize and start the video frame capture
cam = cv2.VideoCapture(0)

# list_of_inputs = [input_1, input_2, input_3, input_4]

encoding_list = [[1,1,1,1], [1,1,1,0], [1,1,0,0], [1,0,0,0],[0,0,0,1], [0,0,1,1]]

list_of_combination = [[1,0,2], [1,2,0], [0,1,2], [2,1,0],[0,2,1], [2,0,1]]

list_of_workers = ["fernando", "dylan", "any person"]

def RaspberryPi_receives_data():
    count = 0
    while True:
        for i in list_of_inputs:
            if i != 0:
                count += 1
        if count > 0:
            break

def raspberry_pi_start():
    if input_5:
        return True
    else:
        return False

count = 0
while True:
    if RaspberryPi_receives_data and count == 0 :
        count +=1
        list_of_inputs = [input_1, input_2, input_3, input_4]
        print(input_1, input_2, input_3, input_4)
        for i in range(len(encoding_list)):
            if collections.Counter(list_of_inputs) == collections.Counter(encoding_list[i]):
                combination_value = i+1
                print("Rpi received the combinations")
                print(combination_value)
        print("=======================START OF THE PROGRAM=======================")
        print(list_of_combination[combination_value - 1])

    while count < 3:
	GPIO.cleanup(input_1)
	GPIO.setup(input_1)
        while True:
            if input_1 == True:
                print("RPI start detecting")
                break
        counter = 0
        while counter == 0:
            GPIO.cleanup(20)
	    GPIO.cleanup(21)
	    GPIO.setup(20, GPIO.OUT)
            GPIO.setup(21, GPIO.OUT)
            # current position
            # Read the video frame
            ret, im =cam.read()
            # Convert the captured frame into grayscale
            gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
            # Get all face from the video frame
            faces = faceCascade.detectMultiScale(gray, 1.2,5)
            # For each face in faces
            for(x,y,w,h) in faces:
                # Create rectangle around the face
                cv2.rectangle(im, (x-20,y-20), (x+w+20,y+h+20), (0,255,0), 4)
                # Recognize the face belongs to which ID
                Id = recognizer.predict(gray[y:y+h,x:x+w])
                # Check the ID if exist 
                if (list_of_combination[combination_value - 1][count] == 2):
                    GPIO.output(20, 1)
                    GPIO.output(21, 1) #to state whether the worker is the right person
                    print("Any person")
                    count +=1
                    counter += 1
                    print(Id[0])
                    break
                else:
                    if(Id[0] == list_of_combination[combination_value - 1][count]):
                        # Id = list_of_workers[position]
                        GPIO.output(20, 1)
                        GPIO.output(21, 1) #to state whether the worker is the right person
                        print("Right person")
                        count +=1
                        print(Id[0])
                        counter += 1
                        break
                        
                    #If not exist, then it is Unknown
                    else:
                        # Id = list_of_workers[2]
                        GPIO.output(20, 1)
                        GPIO.output(21, 0)
                        print("Wrong person")
                        count += 1
                        print(Id[0])
                        counter += 1
                        break
                # # Put text describe who is in the picture
                # cv2.rectangle(im, (x-22,y-90), (x+w+22, y-22), (0,255,0), -1)
                # cv2.putText(im, str(Id), (x,y-40), font, 2, (255,255,255), 3)
            # Display the video frame with the bounded rectangle
            # cv2.imshow('im',im) 
            # If 'q' is pressed, close program
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
# # Stop the camera
# cam.release()
# # Close all windows
# cv2.destroyAllWindows()
