#!/usr/bin/env python3
# *** listener.py ***
# listens for message from talker.py, sends message through TCP socket to
# display_map.py
import rospy
import socket
import pickle
from std_msgs.msg import Float32MultiArray

HOST = '0.0.0.0'
#HOST = socket.gethostbyname('master')
PORT = 8080

def callback(data, conn):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #count1 = int(data.data[0])
    #count2 = int(data.data[1])
    #display = [[0, 0], [1+count1, 1+count2], [2+count2, 2+count1]]
    display = data.data
    display_bytes = pickle.dumps(display)
    conn.sendall(display_bytes)

def listener(conn):
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", Float32MultiArray, callback, conn)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print('Connected by', addr)
            #while True:
            #    data = conn.recv(1024)
            #    if not data:
            #        break
            #    conn.sendall(data)
            listener(conn)
    #listener()
