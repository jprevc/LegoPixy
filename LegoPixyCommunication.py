# -*- coding: utf-8 -*-
"""
Created on Sat Dec 31 13:12:52 2016

@author: miha
"""

def client(iMac, iData = "test", iPort=3):
    import socket

    # The MAC address of a Bluetooth adapter on the server.
    # The server might have multiple Bluetooth adapters.
    serverMACAddress = iMac
    port = iPort  # port is an arbitrary choice. However, it must match the port used by the server.
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    s.connect((serverMACAddress, port))
    text = iData
    s.send(bytes(text, 'UTF-8'))
    s.close()
    
    
    
def server(iMac, iPort = 3, iBacklog = 1, iSize = 1024):
    
    import socket
    
    # The MAC address of a Bluetooth adapter on the server. 
    #The server might have multiple Bluetooth adapters.
    hostMACAddress = iMac 
    port = iPort # port is an arbitrary choice. However, it must match the port used by the client.
    backlog = iBacklog
    size = iSize
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    s.bind((hostMACAddress,port))
    s.listen(backlog)
    try:
        client, address = s.accept()
        # print("Socket is open")
        while 1:
            data = client.recv(size)
            if data:
                print(data)          # print string of data
                client.send(data)
    except:	
        # print("Closing socket")	
        client.close()
        s.close()
    return data
    
    
    
    
def recieveData(iMac, iPort):    
    
    # INPUTS:  iMac:  address of bluetooth on computer which is server    
    #         iPort:  number of port; it must match the port used by the client
    
    
    # OUTPUT:     A:  matrix of output data A = [cIdx dcL dcR posX posY]
    #          cIdx:  color index of block
    #           dcL:  duty cycle of left large motor (EV3)
    #           dcR:  duty cycle of right large motor (EV3)
    #          posX:  x-posotion of block which is detected
    #          posY:  y-posotion of block which is detected
    
    
    import numpy as np    
    
    data = server(iMac, iPort)    
    
    cIdx = data[:1]
    dcL  = data[1:4]
    dcR  = data[4:7]

    
    for i in range(8, len(data)):
        #print(data[i])
        if data[i] == 36:      # 36 = ascii for $
            k = i + 1
            #print(str(k))
            break

    posX = data[8:k - 1]
    posY = data[k:]
    
    A = np.zeros((1,5), dtype = 'float') 
    A[0,0] = cIdx
    A[0,1] = dcL
    A[0,2] = dcR
    A[0,3] = posX
    A[0,4] = posY
    
    """
    print(cIdx)
    print(dcL)
    print(dcR)
    print(posX)
    print(posY)
    """
    
    return A
