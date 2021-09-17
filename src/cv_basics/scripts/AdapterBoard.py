import RPi.GPIO as gp
import wiringpi as wpi
import os
import cv2 as cv 
import numpy as np
import time

class MultiAdapter:
    camNum = 2
    adapter_info = {   "A":{   "i2c_cmd":"i2cset -y 0 0x70 0x00 0x04",
                                    "gpio_sta":[0,0,1],
                            },
                        "B":{
                                "i2c_cmd":"i2cset -y 0 0x70 0x00 0x06",
                                "gpio_sta":[1,0,1],
                            },
                     } 
    camera = cv.VideoCapture(0) 
    width = 320
    height = 240 

    font = cv.FONT_HERSHEY_PLAIN
    fontScale = 1
    fontColor = (255, 255, 255)
    lineType = 1
    factor = 20
    black = np.zeros(((height+factor)*2, width*2, 3), dtype= np.uint8) 
        

    def __init__(self):
       """
       gp.setwarnings(False)
       gp.setmode(gp.BOARD)
       gp.setup(7, gp.OUT)
       gp.setup(11,gp.OUT)
       gp.setup(12,gp.OUT)
       """

       print("gets in Adapter")
       wpi.wiringPiSetup()
       wpi.pinMode(7, 1)    # set pin 7 to 1 (OUTPUT)
       wpi.pinMode(11, 1)   # set pin 11 to 1 (OUTPUT)
       wpi.pinMode(12, 1)   # set pin 12 to 1 (OUTPUT)


    def choose_channel(self,index):
        channel_info = self.adapter_info.get(index)
        if channel_info == None:
            print("Can't get this info")
        os.system(channel_info["i2c_cmd"]) # i2c write
        gpio_sta = channel_info["gpio_sta"] # gpio write
        """
        gp.output(7, gpio_sta[0])
        gp.output(11, gpio_sta[1])
        gp.output(12, gpio_sta[2])
        """
        wpi.digitalWrite(7, gpio_sta[0])
        wpi.digitalWrite(11, gpio_sta[1])
        wpi.digitalWrite(12, gpio_sta[2])

    def select_channel(self,index):
        channel_info = self.adapter_info.get(index)
        if channel_info == None:
            print("Can't get this info")
        gpio_sta = channel_info["gpio_sta"] # gpio write
        """
        gp.output(7, gpio_sta[0])
        gp.output(11, gpio_sta[1])
        gp.output(12, gpio_sta[2])
        """
        wpi.digitalWrite(7, gpio_sta[0])
        wpi.digitalWrite(11, gpio_sta[1])
        wpi.digitalWrite(12, gpio_sta[2])

    def init(self,width,height):
        for i in range(self.camNum):
           self.height = height
           self.width = width
           self.choose_channel(chr(65+i)) 
           self.camera.set(3, self.width)
           self.camera.set(4, self.height)
           ret, frame = self.camera.read()
           if ret == True:
               print("camera %s init OK" %(chr(65+i)))
               pname = "image_"+ chr(65+i)+".jpg"
               cv.imwrite(pname,frame)
               time.sleep(1)
    
    def get_snapshot(self, camera_ID):
        self.select_channel(chr(65+camera_ID))
        ret, frame = self.camera.read()
        ret, frame = self.camera.read()
        ret, frame = self.camera.read()
        frame.dtype=np.uint8

        #self.black[self.factor:self.factor+self.height, 0:self.width, :] = frame
        #bottomLeftCornerOfText = (self.factor, self.factor)
        #index = chr(65+i)
        #cv.putText(black,'CAM '+index, bottomLeftCornerOfText, font, fontScale,fontColor,lineType)
        #cv.imshow("Arducam Multi Camera Demo",black)
        #if cv.waitKey(1) & 0xFF == ord('q'):
        #    del frame
        #    self.camera.release()
        #    cv.destroyAllWindows()
        #    break
        return ret, frame


    def preview(self):
        font                   = cv.FONT_HERSHEY_PLAIN
        fontScale              = 1
        fontColor              = (255,255,255)
        lineType               = 1
        factor  = 20
        black = np.zeros(((self.height+factor)*2, self.width*2, 3), dtype= np.uint8) 
        i = 0
        while True:
            self.select_channel(chr(65+i)) 
            ret, frame = self.camera.read()
            ret, frame = self.camera.read()
            ret, frame = self.camera.read()
            frame.dtype=np.uint8
            if i == 0:
                black[factor:factor+self.height, 0:self.width, :] = frame
                bottomLeftCornerOfText = (factor,factor)
                index = chr(65+i)
            elif i == 1:
                black[factor:factor+self.height, self.width:self.width*2,:] = frame
                bottomLeftCornerOfText = (factor+self.width, factor)
                index = chr(65+i)
            elif i == 2:
                black[factor*2+self.height:factor*2+self.height*2, 0:self.width,:] = frame
                bottomLeftCornerOfText = (factor, factor*2+self.height)
                index = chr(65+i)
            elif i == 3:
                black[factor*2+self.height:factor*2+self.height*2, self.width:self.width*2,:] = frame
                bottomLeftCornerOfText = (factor+self.width, factor*2+self.height)
                index = chr(65+i)
            i = i+1
            if i==self.camNum:
                i = 0
            cv.putText(black,'CAM '+index, bottomLeftCornerOfText, font, fontScale,fontColor,lineType)
            cv.imshow("Arducam Multi Camera Demo",black)
            if cv.waitKey(1) & 0xFF == ord('q'):
                del frame
                self.camera.release()
                cv.destroyAllWindows()
                break


        

