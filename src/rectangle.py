import cv2
import numpy as np
import serial.tools.list_ports
import time


#Declarations
kp = 60
ki = 0; 
kd = -30
cm = -1
eight_move = -1

xp = 0;
yp = 0;
wp = 1;
hp = 0;
xb = 0;
yb = 0;
wb = 1;
hb = 0;
xs = 0
ys = 0
last_time = 0;
#For detecting platform color
lower = np.array([200 / 2, 0.05 * 255, 0.4 * 255])   #[205 / 2, 0.33 * 255, 0.45 * 255]
upper = np.array([240 / 2, 0.75 * 255, 1 * 255])   #[215 / 2, 0.54 * 255, 0.98 * 255]

lower_green = np.array([130 / 2, 0.30 * 255, 0.35 * 255])   #[205 / 2, 0.33 * 255, 0.45 * 255]
upper_green = np.array([180 / 2, 1 * 255, 1 * 255]) 
def write_ser(serialInst, cmd):
    cmd = cmd + '\n'
    serialInst.write(cmd.encode())

def open_USB_Connection(serialInst):
    ports = serial.tools.list_ports.comports()
    portList = []
    for port in ports:
        portList.append(str(port))
        print(str(port))
    val = 8 #input("select COM:")
    portName = "COM" + str(val)
    serialInst.baudrate = 500000
    serialInst.port = portName
    serialInst.open()

def send_coordinates(serialInst, xxx, yyy):
    str_command = ("%.4f,%.4f,%.1f,%.1f,%.1f,%.1f,%.1f" % (xxx, yyy, kp, ki, kd, cm, eight_move))
    write_ser(serialInst, str_command)
    

########## START ##########
serialInst = serial.Serial()  #initialize serial
open_USB_Connection(serialInst) #open connection with ESP32
serialInst.flush()
video = cv2.VideoCapture(1)

while True:
    counter=0
    while(time.time() - last_time < 0.03):
        counter+=1
        continue
    print(counter)
    last_time = time.time()
    success, img = video.read()

    #Cropping  
    img = img[10:len(img)-10, 170:len(img[0])-120, :]
    img = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)

    #Platform
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lower, upper)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #print(contours)
    if contours is not None:
        for contour in contours:
            #Finding large contour area
            if cv2.contourArea(contour) >= 10000:
                xp, yp, wp, hp = cv2.boundingRect(contour)
                cv2.rectangle(img, (xp, yp), (xp+wp, yp+hp), (0,255,0), 3)

    #Circle
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #gray = cv2.GaussianBlur(gray, (5,5), 0)
    #Finding bright object (ball)
    # WHITE #_, mask_circ = cv2.threshold(gray, 195, 255, cv2.THRESH_BINARY)
    mask_circ = cv2.inRange(hsv_img, lower_green, upper_green)
    contours, hierarchy = cv2.findContours(mask_circ,  cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours is not None:
       for contour in contours:
            #Finding object of correct size
            if cv2.contourArea(contour) >= 25 and cv2.contourArea(contour) < 3000:
                xb, yb, wb, hb = cv2.boundingRect(contour)
                cv2.rectangle(img, (xb, yb), (xb+wb, yb+hb), (0,0,255), 3)
        
        

    
    #Calculate position of Ball
    if wp!=0 and hp!=0:
        xs = ((xb - xp) + wb/2) / wp
        ys = ((yb - yp) + hb/2) / hp
    #print("X scale = " + str(ys))
    
    #Send to ESP32
    if xs > 0 and ys > 0 or False:
        send_coordinates(serialInst, xs, ys)        
        #read_ser(serialInst)
        packet = serialInst.readline()
        print(packet)
        print(time.time()-last_time)

    

    cv2.imshow("cam", img)
    cv2.imshow("mask", mask)
    #cv2.imshow("gray", gray)
    cv2.imshow("mask circ", mask_circ)
    
    #print(time.time() - last_time)
    #cv2.waitKey(1)
    key = cv2.waitKey(1)
    if key == ord('p'):
        kp = float(input("new kp = "))
    elif key == ord('i'):
        ki = float(input("new ki = "))
    elif key == ord('d'):
        kd = float(input("new kd = "))
    elif key == ord('c'):
        cm = float(input("new c = "))
        eight_move = -1
    elif key == ord('8'):
        eight_move = float(input("new 8 = "))
        cm = -1