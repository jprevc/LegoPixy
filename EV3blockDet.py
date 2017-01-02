
"""
funkction for communication with bluetooth

"""

def client(iMac, iData = "test", iPort=3):
    import socket

    # The MAC address of a Bluetooth adapter on the server.
    # The server might have multiple Bluetooth adapters.
    serverMACAddress = iMac
    port = iPort  # port is an arbitrary choice. However, it must match the port used by the server.
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    try:
        s.connect((serverMACAddress, port))
        text = iData
        s.send(bytes(text, 'UTF-8'))
        s.close()
    except:
        s.close()



def server(iMac, iPort=3, iBacklog=1, iSize=1024):
    import socket

    # The MAC address of a Bluetooth adapter on the server.
    # The server might have multiple Bluetooth adapters.
    hostMACAddress = iMac
    port = iPort  # port is an arbitrary choice. However, it must match the port used by the client.
    backlog = iBacklog
    size = iSize
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    s.bind((hostMACAddress, port))
    s.listen(backlog)
    try:
        client, address = s.accept()
        print("Socket is open")
        while 1:
            data = client.recv(size)
            if data:
                print("Data ready")
                client.send(data)
    except:
        print("Closing socket")
        client.close()
        # data = 0
        s.close()
    return data



"""
Created on Sun Jan  1 17:34:26 2017
@author: jostp
"""


def computeObjectPosition(iObjectWidthPx, iObjectXcoordPx, iObjectWidth=64,  iFocalLength=215, iCamMaxAngle=38.65):
    '''
    Returns detected object's position in camera coordinate system, according to his width, and its center's x coordinate

    Parameters
    -----------
    iObjectWidthPx : int
        Object's width in pixels
    iObjectWidth : float
        Object's width in mm
    iObjectXcoordPx : int
        Object's x coordinate on camera in pixels
    iFocalLength : float
        Focal length of the camera in mm
    iCamMaxAngle : float
        Maximum camera vision angle in degrees

    Returns
    ----------
    np.array
        Computed postion (x,y) of object in camera coordinate system

    '''

    import numpy as np

    # compute object's distance from camera
    if iObjectWidthPx != 0:
        distance = iObjectWidth * iFocalLength / iObjectWidthPx
    else:
        distance = 0


    # compute object's angle to camera
    koef = -iCamMaxAngle / 128.0
    objectAngle = koef * (iObjectXcoordPx - 128)

    # compute object's coordinates
    oX = distance * np.cos(objectAngle * np.pi / 180)
    oY = distance * np.sin(objectAngle * np.pi / 180)


    return np.array((oX, oY))

#########################################################################################################
####################################### programm on EV3  ################################################
#########################################################################################################


import time
import ev3dev.ev3 as ev3

pixy = ev3.Sensor('in1')
pixy.mode = 'ALL'

m_left = ev3.LargeMotor('outB')
m_right = ev3.LargeMotor('outC')

btn = ev3.Button()


while not btn.any():

    dc_l = 20

    dc_r = 10

    # set motor speed values
    #m_left.run_direct(duty_cycle_sp=dc_l)
    #m_right.run_direct(duty_cycle_sp=dc_r)

    a = pixy.value(0)
    #b = pixy.value(1)
    c = pixy.value(2)
    #d = pixy.value(3)
    e = pixy.value(4)
    #f = pixy.value(5)

    pose = computeObjectPosition(e, c)

    aS = str(a)
    posX = str(pose[0])
    posY = str(pose[1])
    dcL  = str(dc_l)
    dcR  = str(dc_r)

    if len(aS) != 1:
        aS = "0"

    if len(dcL) == 1:
        dcL = "00" + dcL
    elif len(dcL) == 2:
        dcL = "0" + dcL
    else:
        dcL = dcL


    if len(dcR) == 1:
        dcR = "00" + dcR
    elif len(dcR) == 2:
        dcR = "0" + dcR
    else:
        dcR = dcR


    text = aS + dcL + dcR + '$' + posX + '$' + posY

    time.sleep(0.1)
    client('a4:db:30:56:59:71',text, 4)



# stop both motors
#m_left.stop()
#m_right.stop()
