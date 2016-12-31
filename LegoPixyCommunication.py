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
                print(data)
                client.send(data)
    except:	
        # print("Closing socket")	
        client.close()
        s.close()
    return data
