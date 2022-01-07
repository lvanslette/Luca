#!/usr/bin/env python3

import socket
import time

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 8000        # The port used by the server
serverAddressPort = (HOST, PORT)
bufferSize = 1024
msgFromServer = "Hello UDP server"
bytesToSend = str.encode(msgFromServer)

sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
while True:
  sock.sendto(bytesToSend, serverAddressPort)
  msgFromServer = sock.recvfrom(bufferSize)
  msg = f"Message from server: {msgFromServer[0]}"
  print(msg)
  time.sleep(1)
