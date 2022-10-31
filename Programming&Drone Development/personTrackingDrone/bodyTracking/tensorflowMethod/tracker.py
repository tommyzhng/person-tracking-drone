# Based off of 
# https://github.com/tensorflow/tensorflow/blob/master/tensorflow/lite/examples/python/label_image.py
import importlib
from threading import Thread
import cv2 as cv
import numpy as np
import os

class GetVideo(): #get video from another thread to reduce latency
    def __init__(self) -> None:
        self.stream = cv.VideoCapture(0)
        (self.success, self.frame) = self.stream.read() #read the first frame
        self.stopped = False

    def start(self):
        Thread(target=self.getframes, args=()).start() #start a thread to read frames
        return self

    def getframes(self):
        while not self.stopped:
            if not self.success: ##if cant grab frame, then stop the thread
                self.stop()
            else:
                (self.success, self.frame) = self.stream.read()
    def stop(self):
        self.stopped = True


class Tracker():
    def __init__(self) -> None:
        #if this is running on raspi, use import from rflite runtime, or else use windows interpreter
        if importlib.util.find_spec('tflite_runtime'):
            from tflite_runtime.interpreter import Interpreter
        else:
            from tensorflow.lite.python.interpreter import Interpreter

        CWD = os.getcwd()
        MODELPATH = os.path.join(CWD,"model",'detect.tflite')
        LABELPATH = os.path.join(CWD,"model", 'labelmap.txt')

        #get rid of "\n" when reading label files
        self.labels = [line.strip() for line in (open(LABELPATH, 'r')).readlines()]
        
        #load tensor interpreter
        self.interpreter = Interpreter(model_path=MODELPATH)
        self.interpreter.allocate_tensors()

        self.input_det = self.interpreter.get_input_details()
        self.output_det = self.interpreter.get_output_details()
        #height and width from input det: "'shape': array([  1, 300, 300,   3])"
        self.height = 300
        self.width = 300
        
        self.frame_rate_calc = 1
        self.freq = cv.getTickFrequency()  

    def process(self, frame):
        frame = cv.resize(frame, (640,480))
        frame = cv.flip(frame, 1)
        frame = cv.rotate(frame, cv.ROTATE_90_CLOCKWISE)
        t1 = cv.getTickCount() #initial frame
        
        framergb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)     ##create another frame to process on
        frame1 = cv.resize(framergb, (self.width, self.height))
        
        data = np.expand_dims(frame1, axis=0)
        self.interpreter.set_tensor(self.input_det[0]['index'],data)
        self.interpreter.invoke()

        #Bounding box coordinates
        bbox = self.interpreter.get_tensor(self.output_det[0]['index'])[0]
        #Class index
        classes = self.interpreter.get_tensor(self.output_det[1]['index'])[0] 
        #Confidence level
        confidence = self.interpreter.get_tensor(self.output_det[2]['index'])[0] 

        center = (0,0)
        area = 0

        if (self.labels[int(classes[0])] == "person"): #run if detected person
            if (confidence[0] > 0.4 and confidence[0] < 1.0): #if confidence is high, run the bounding boxes
                    minx, maxx = int(max(1,(bbox[0][1] * 480))), int(min(640,(bbox[0][3] * 480)))
                    miny, maxy = int(max(1,(bbox[0][0] * 640))), int(min(640,(bbox[0][2] * 640)))
                    center = round((maxx + minx)/2, 3), round((maxy + miny)/2, 3)

                    cv.circle(frame, (int(center[0]), int(center[1])), 15, (0, 0, 255), -1)
                    cv.rectangle(frame, (minx,miny), (maxx, maxy), (0,255,0), 2)
                    area = abs(100*((maxy/640)-(miny/640)))


        differences = self.distanceFromCenter(frame, center)
        cv.putText(frame,f'FPS: {round(self.frame_rate_calc)}',(30,50),cv.FONT_HERSHEY_SIMPLEX,1,(0,255,0),1,cv.LINE_AA)
        
        #calculate frame rate for next loop
        t2 = cv.getTickCount()
        time1 = (t2-t1)/self.freq
        self.frame_rate_calc= 1/time1

        return frame, differences, area

    def distanceFromCenter(self, frame, center):
            #Center
            if center != (0, 0):
                #Draw circle at center of screen
                cv.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), 15, (255, 0, 0), -1)

                #Draw a line connecting the two points
                cv.line(frame, (240, 320), (int(center[0]), int(center[1])), (255, 255, 0), 10)

                #Find X Y differences           only need #X value for yaw
                differences = ((0.5 - center[0]/480))
                cv.putText(frame, f"X Delta = {round(differences * 100, 2)}%", (140, 360), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                differences = 0

            return differences
    

        


