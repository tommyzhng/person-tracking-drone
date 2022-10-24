# Based off of 
# https://github.com/tensorflow/tensorflow/blob/master/tensorflow/lite/examples/python/label_image.py
import importlib
import cv2 as cv
import os
class Tracker():
    def __init__(self) -> None:
        self.capture = cv.VideoCapture(0)

        if importlib.util.find_spec('tflite_runtime'):
            from tflite_runtime.interpreter import Interpreter
        else:
            from tensorflow.lite.python.interpreter import Interpreter

        CWD = os.getcwd()
        MODELPATH = os.path.join(CWD,"model",'detect.tflite')
        LABELPATH = os.path.join(CWD,"model", 'labelmap.txt')

        interpreter = Interpreter(model_path=MODELPATH)
        interpreter.allocate_tensors()
    def read(self):
        self.success, self.frame = self.capture.read()
        self.frame = self.process()
        return self.frame
    def process():
        pass


