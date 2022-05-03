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

serialInst = serial.Serial()
open_USB_Connection(serialInst)
serialInst.flush()

camera = cv2.VideoCapture(1)

while True:
    ret, src = camera.read()
    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    rows = gray.shape[0]
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                              param1=100, param2=30,
                              minRadius=15, maxRadius=60)
    """
    if cv2.waitKey(5) == ord('c'):
        print('Im in message center.');
        str_comand = input()
        write_ser(str_comand)
        #while(serialInst.rea)
        packet = serialInst.read(5)
        print(packet.decode())
        serialInst.flush()
    """


    #print(i);
    #i = i + 1;

    if circles is not None:
        circles = np.uint16(np.around(circles))
        send_coordinates(circles[0][0][0], circles[0][0][1])
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(src, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(src, center, radius, (255, 0, 255), 3)

    cv2.imshow("detected circles", src)

    if cv2.waitKey(1) == ord('q'):
        break

print("Im out of main loop!")
camera.release()

cv2.destroyAllWindows()

