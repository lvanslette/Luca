#!/usr/bin/env python3

import socket

HOST = '192.168.1.1'  # The server's hostname or IP address
PORT = 8000        # The port used by the server
serverAddressPort = (HOST, PORT)
bufferSize = 1024
msgFromServer = "Hello UDP client"
bytesToSend = str.encode(msgFromServer)

sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# host binds to host, port
sock.bind((HOST, PORT))
print("UDP server up and listening")

while True:
  byteAddressPair = sock.recvfrom(bufferSize)
  message = byteAddressPair[0]
  address = byteAddressPair[1]
  clientMsg = "message from client:{}".format(message)
  clientIP = "client IP address:{}".format(address)
  print(clientMsg)
  print(clientIP)

  sock.sendto(bytesToSend, address)
