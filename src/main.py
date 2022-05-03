import cv2
import numpy as np
import serial.tools.list_ports

def write_ser(cmd):
    cmd = cmd + '\n'
    serialInst.write(cmd.encode())

def open_USB_Connection(serialInst):
    ports = serial.tools.list_ports.comports()

    portList = []
    for port in ports:
        portList.append(str(port))
        print(str(port))

    val = input("select COM:")
    portName = "COM" + str(val)

    serialInst.baudrate = 115200
    serialInst.port = portName
    serialInst.open()


def send_coordinates(xxx, yyy):
    str_comand = str(xxx) + str(yyy)
    write_ser(str_comand)
    # while(serialInst.rea)
    #packet = serialInst.read(6)
    #print(packet.decode())
    print(str_comand)
    serialInst.flush()

def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )



serialInst = serial.Serial()
open_USB_Connection(serialInst)
serialInst.flush()

camera = cv2.VideoCapture(1)

while True:
    ret, src = camera.read()
    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.medianBlur(gray, 5)
    rows = gray2.shape[0]

    #nakładanie masek i wykrywanie konturów
    blurred_frame = cv2.GaussianBlur(src, (5, 5), 0)
    hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
    print(hsv[300][300])
    upper_blue = np.array([115, 70, 255]) #253, 253, 253
    lower_blue = np.array([0, 0, 70]) #193, 201, 160

    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    _, contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    #ctr = np.array(contours).reshape((-1,1,2)).astype(np.int64)
    #cv2.drawContours(src, [ctr], 0, (0, 255, 0), -1)
        #for contour in contours:
            #cv2.drawContours(src, contour, 0, (0, 255, 0), 3)
            

    circles = cv2.HoughCircles(gray2, cv2.HOUGH_GRADIENT, 1, rows / 8,
                              param1=100, param2=30,
                              minRadius=15, maxRadius=60)
    

    """                
    canny = cv2.Canny(gray, 130, 255, 1)
    cnts = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    
    for c in cnts:
        print("wykryto krawedz")
        cv2.drawContours(src,[c], 0, (0,255,0), 3)
    """
    """
    #find threshold of the image
    _, thrash = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    for contour in contours:
        shape = cv2.approxPolyDP(contour, 0.15*cv2.arcLength(contour, True), True)
        x_cor = shape.ravel()[0]
        y_cor = shape.ravel()[1]
        
        if len(shape) ==4:
            #shape cordinates
            x,y,w,h = cv2.boundingRect(shape)

            #width:height
            aspectRatio = float(w)/h
            cv2.drawContours(src, [shape], 0, (0,255,0), 4)
            if aspectRatio >= 0.9 and aspectRatio <=1.1:
                cv2.putText(src, "Square", (x_cor, y_cor), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,0,0))
            else:
                cv2.putText(src, "Rectangle", (x_cor, y_cor), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255,0,0))
    """
    if circles is not None:
        circles = np.uint16(np.around(circles))
        #send_coordinates(circles[0][0][0], circles[0][0][1])
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(src, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(src, center, radius, (255, 0, 255), 3)

    cv2.imshow("detected circles", src)
    cv2.imshow("Mask", mask)
    if cv2.waitKey(1) == ord('q'):
        break

print("Im out of main loop!")
camera.release()

cv2.destroyAllWindows()

