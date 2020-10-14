import cv2
import imutils
import numpy as np
import pytesseract
from PIL import Image
from picamera.array import PiRGBArray
from picamera import PiCamera
import smtplib
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
import RPi.GPIO as GPIO
import time
import threading
echo1 = 16
trg1 = 18
echo2 = 11
trg2 =13
echo3 = 35
trg3 = 37
echo4 = 38
trg4 = 40
Spin = 3 #Servo pin Entry gate
Spin2 = 20 #Servo pin Exit gate
IR = 7 #IR sensor for entry
IR2 = 33 #IR sensor for exit
GPIO.setmode(GPIO.BOARD)

GPIO.setup(echo1,GPIO.IN)
GPIO.setup(trg1,GPIO.OUT)
GPIO.setup(echo2,GPIO.IN)
GPIO.setup(trg2,GPIO.OUT)
GPIO.setup(echo3,GPIO.IN)
GPIO.setup(trg3,GPIO.OUT)
GPIO.setup(echo4,GPIO.IN)
GPIO.setup(trg4,GPIO.OUT)
GPIO.setup(Spin,GPIO.OUT)
GPIO.setup(Spin2,GPIO.OUT)
GPIO.setup(IR,GPIO.IN)
GPIO.setup(IR2,GPIO.IN)
pwm = GPIO.PWM(Spin,50)
pwm2 = GPIO.PWM(Spin2,50)
pwm.start(3)
pwm2.start(3)

cred = credentials.Certificate('./serviceAccount.json')
firebase_admin.initialize_app(cred)
db = firestore.client()
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))
def capt_Entry():
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        if key == ord("s"):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #convert to grey scale
            gray = cv2.bilateralFilter(gray, 11, 17, 17) #Blur to reduce noise
            edged = cv2.Canny(gray, 30, 200) #Perform Edge detection
            cnts = cv2.findContours(edged.copy(), cv2.RETR_TREE,              cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
            screenCnt = None
            for c in cnts:
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.018 * peri, True)
                if len(approx) == 4:
                    screenCnt = approx
                    break
            if screenCnt is None:
                detected = 0
                print ("No contour detected")
            else:
            	detected = 1

            if detected == 1:
                cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 3)
               	mask = np.zeros(gray.shape,np.uint8)
               	new_image = cv2.drawContours(mask,[screenCnt],0,255,-1,)
               	new_image = cv2.bitwise_and(image,image,mask=mask)
               	(x, y) = np.where(mask == 255)
               	(topx, topy) = (np.min(x), np.min(y))
               	(bottomx, bottomy) = (np.max(x), np.max(y))
               	Cropped = gray[topx:bottomx+1, topy:bottomy+1]
               	text = pytesseract.image_to_string(Cropped, config='--psm 11')
               	text = text.rstrip()
                #Fetching License Plate No. from DB
               	cars_ref = db.collection(u'Car Number')
               	docs = cars_ref.stream()
               	allLPN = []
               	for doc in docs:
                    allLPN.append((doc.to_dict())["LPN"])
                    print(allLPN)
               		print(text)
               	if text in allLPN:
               	 	print("Detected Number is:",text)
                    pwm.ChangeDutyCycle(7.5)
                    time.sleep(5)
                else:
                 	pwm.ChangeDutyCycle(3)
               	return cv2.imshow("Frame", image)
              	return cv2.imshow('Cropped',Cropped)
               	return text
               	cv2.waitKey(0)
               	break
cv2.destroyAllWindows()

def capt_Exit():
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        if key == ord("s"):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #convert to grey scale
            gray = cv2.bilateralFilter(gray, 11, 17, 17) #Blur to reduce noise
            edged = cv2.Canny(gray, 30, 200) #Perform Edge detection
            cnts = cv2.findContours(edged.copy(), cv2.RETR_TREE,              cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
            screenCnt = None
            for c in cnts:
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.018 * peri, True)
                if len(approx) == 4:
                    screenCnt = approx
                    break
            if screenCnt is None:
                detected = 0
                print ("No contour detected")
            else:
            	detected = 1

            if detected == 1:
                cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 3)
               	mask = np.zeros(gray.shape,np.uint8)
               	new_image = cv2.drawContours(mask,[screenCnt],0,255,-1,)
               	new_image = cv2.bitwise_and(image,image,mask=mask)
               	(x, y) = np.where(mask == 255)
               	(topx, topy) = (np.min(x), np.min(y))
               	(bottomx, bottomy) = (np.max(x), np.max(y))
               	Cropped = gray[topx:bottomx+1, topy:bottomy+1]
               	text = pytesseract.image_to_string(Cropped, config='--psm 11')
               	text = text.rstrip()
                #Fetching License Plate No. from DB
               	cars_ref = db.collection(u'Car Number')
               	docs = cars_ref.stream()
               	allLPN = []
               	for doc in docs:
                    allLPN.append((doc.to_dict())["LPN"])
                    print(allLPN)
               		print(text)
               	if text in allLPN:
               	 	print("Detected Number is:",text)
                    pwm2.ChangeDutyCycle(7.5)
                    time.sleep(5)
                else:
                 	pwm2.ChangeDutyCycle(3)
               	return cv2.imshow("Frame", image)
              	return cv2.imshow('Cropped',Cropped)
               	return text
               	cv2.waitKey(0)
               	break
cv2.destroyAllWindows()

def ultra1():
    global x
    while True:
        GPIO.output(trg1,0)
        
        time.sleep(2)

        GPIO.output(trg1,1)
        time.sleep(0.00001)
        GPIO.output(trg1,0)
        
        while(GPIO.input(echo1)) == 0:
            start = time.time()
        while(GPIO.input(echo1)) == 1:
            stop =time.time()

        duration = stop - start

        dist = duration*34300./2
        dist = round(dist,2)      
        #print("Distance of 1st ultrasonic is: {} cm ".format(dist))
        if dist <9.5: #If distance in less than 9.5 cm then we will get 1 hence red colour on the app
            x=1
        if dist >9.5:
            x=0
        cars_ref = db.collection(u'cars')
        docs = cars_ref.stream()
        car_ref = db.collection(u'cars').document(u'Car 1').get()
        y = car_ref.to_dict()["Status"]
        if x!=y:
            doc_ref = db.collection(u'cars').document('Car 1')
            doc_ref.set({u'Status': x })            

def ultra2():
    global x2
    while True:
        GPIO.output(trg2,0)
        
        time.sleep(2)

        GPIO.output(trg2,1)
        time.sleep(0.00001)
        GPIO.output(trg2,0)
        
        while(GPIO.input(echo2)) == 0:
            start = time.time()
        while(GPIO.input(echo2)) == 1:
            stop =time.time()

        duration = stop - start

        dist = duration*34300./2
        dist = round(dist,2)      
        #print("Distance of 2nd ultrasonic is: {} cm ".format(dist))
        if dist <9.5:
            x2=1
        if dist >9.5:
            x2=0
        cars_ref = db.collection(u'cars')
        docs = cars_ref.stream()
        car_ref = db.collection(u'cars').document(u'Car 2').get()
        y = car_ref.to_dict()["Status"]
        if x2!=y:
            doc_ref = db.collection(u'cars').document('Car 2')
            doc_ref.set({u'Status': x2 })  



def ultra3():
    global x3
    while True:
        GPIO.output(trg3,0)
        
        time.sleep(2)

        GPIO.output(trg3,1)
        time.sleep(0.00001)
        GPIO.output(trg3,0)
        
        while(GPIO.input(echo3)) == 0:
            start = time.time()
        while(GPIO.input(echo3)) == 1:
            stop =time.time()

        duration = stop - start

        dist = duration*34300./2
        dist = round(dist,2)      
        #print("Distance of 3rd ultrasonic is: {} cm ".format(dist))
        if dist <9.5:
            x3=1
        if dist >9.5:
            x3=0
        cars_ref = db.collection(u'cars')
        docs = cars_ref.stream()
        car_ref = db.collection(u'cars').document(u'Car 3').get()
        y = car_ref.to_dict()["Status"]
        if x3!=y:
            doc_ref = db.collection(u'cars').document('Car 3')
            doc_ref.set({u'Status': x3 })  



def ultra4():
    global x4
    while True:
        GPIO.output(trg4,0)
        
        time.sleep(2)

        GPIO.output(trg4,1)
        time.sleep(0.00001)
        GPIO.output(trg4,0)
        
        while(GPIO.input(echo4)) == 0:
            start = time.time()
        while(GPIO.input(echo4)) == 1:
            stop =time.time()

        duration = stop - start

        dist = duration*34300./2
        dist = round(dist,2)      
        #print("Distance of 4th ultrasonic is: {} cm ".format(dist))
        if dist <9.5:
            x4=1
        if dist >9.5:
            x4=0
        cars_ref = db.collection(u'cars')
        docs = cars_ref.stream()
        car_ref = db.collection(u'cars').document(u'Car 4').get()
        y = car_ref.to_dict()["Status"]

        if x4!=y:
            doc_ref = db.collection(u'cars').document('Car 4')
            doc_ref.set({u'Status': x4 })  


def IR_Entry():
    while True:
        IR_Val = GPIO.input(IR)
        if IR_Val == 0:
            capt_Entry()
        if IR_Val == 1:
            pwm.ChangeDutyCycle(3)
            cv2.destroyAllWindows()
def IR_Exit():
    while True:
        IR_Val = GPIO.input(IR2)
        if IR_Val == 0:
            capt_Exit()
        if IR_Val == 1:
            pwm2.ChangeDutyCycle(3)
            cv2.destroyAllWindows()

def full(): #If all the parking slots are unavailable then the Entry gate will not open
	while True:
		if x==1 and x2==1 and x3==1 and x4 ==1:
			pwm.ChangeDutyCycle(3)
			print("Parking slots are full")

t1 = threading.Thread(target = ultra1,daemon=True)
t2 = threading.Thread(target = ultra2,daemon=True)
t3 = threading.Thread(target = ultra3,daemon=True)
t4 = threading.Thread(target = ultra4,daemon=True)
t5 = threading.Thread(target = IR_Entry)
t6 = threading.Thread(target = IR_Exit)
t7 = threading.Thread(target = full)

t1.start()
t2.start()
t3.start()
t4.start()
t5.start()
t6.start()
t7.start()

#IF the code doesnt work try removing the IR2 ,pwm2 and capt_exit parameters and then try again