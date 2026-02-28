# -*- coding: utf-8 -*-
"""
Created on Sat Dec 31 13:12:52 2016

@author: miha
"""

from legopixy.constants import ASCII_DOLLAR


def client(mac_address, data_payload="test", port=3):
    import socket

    # The MAC address of a Bluetooth adapter on the server.
    # The server might have multiple Bluetooth adapters.
    server_mac_address = mac_address
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    try:
        s.connect((server_mac_address, port))
        text = data_payload
        s.send(bytes(text, "UTF-8"))
    except OSError:
        pass  # Connection/socket error; ensure socket is closed
    finally:
        s.close()


def server(mac_address, port=3, backlog=1, size=1024):

    import socket

    # The MAC address of a Bluetooth adapter on the server.
    # The server might have multiple Bluetooth adapters.
    host_mac_address = mac_address
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    s.bind((host_mac_address, port))
    s.listen(backlog)
    client_socket = None
    try:
        client_socket, address = s.accept()
        # print("Socket is open")
        while True:
            data = client_socket.recv(size)
            if data:
                print(data)  # print string of data
                client_socket.send(data)
    except OSError:
        # print("Closing socket")
        if client_socket is not None:
            client_socket.close()
        s.close()
        raise


def receive_data(mac_address, port):

    # INPUTS:  iMac:  address of bluetooth on computer which is server
    #         iPort:  number of port; it must match the port used by the client

    # OUTPUT:     A:  matrix of output data A = [cIdx dcL dcR posX posY]
    #          cIdx:  color index of block
    #           dcL:  duty cycle of left large motor (EV3)
    #           dcR:  duty cycle of right large motor (EV3)
    #          posX:  x-position of block which is detected
    #          posY:  y-position of block which is detected

    import numpy as np

    data = server(mac_address, port)

    color_index = data[:1]
    dc_left = data[1:4]
    dc_right = data[4:7]

    for i in range(8, len(data)):
        # print(data[i])
        if data[i] == ASCII_DOLLAR:
            split_index = i + 1
            # print(str(k))
            break

    pos_x = data[8 : split_index - 1]
    pos_y = data[split_index:]

    output_data = np.zeros((1, 5), dtype="float")
    output_data[0, 0] = color_index
    output_data[0, 1] = dc_left
    output_data[0, 2] = dc_right
    output_data[0, 3] = pos_x
    output_data[0, 4] = pos_y

    """
    print(cIdx)
    print(dcL)
    print(dcR)
    print(posX)
    print(posY)
    """

    return output_data
