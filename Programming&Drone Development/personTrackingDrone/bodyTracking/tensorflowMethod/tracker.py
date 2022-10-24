# Based off of 
# https://github.com/tensorflow/tensorflow/blob/master/tensorflow/lite/examples/python/label_image.py
import importlib
from threading import Thread
import cv2 as cv
import numpy as np
import os


class Tracker():
    def __init__(self) -> None:
        ###### start video capture ######
        self.capture = cv.VideoCapture(0)
        #################################
        
        #if this is running on raspi, use import from rflite runtime
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
        #height and width from "'shape': array([  1, 300, 300,   3])"
        self.height = 300
        self.width = 300
        
        self.frame_rate_calc = 1
        self.freq = cv.getTickFrequency()  
 
    def read(self):
        success, frame = self.capture.read()
        return frame

    def process(self, frame):
        frame = cv.flip(frame, 1)
        t1 = cv.getTickCount()
        
        framergb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
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
        
        for i in range(len(confidence)):
            if (self.labels[int(classes[i])] == "person"):
                if (confidence[i] > 0.6 and confidence[i] < 1.0): #if confidence is high, run the bounding boxes
                    pass      

        cv.putText(frame,f'FPS: {round(self.frame_rate_calc)}',(30,50),cv.FONT_HERSHEY_SIMPLEX,1,(0,255,0),1,cv.LINE_AA)
        t2 = cv.getTickCount()
        time1 = (t2-t1)/self.freq
        self.frame_rate_calc= 1/time1

        return frame
    

        


