import cv2 as cv
import mediapipe as mp

class HandDetector:
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, minTrackCon=0.5):

        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.minTrackCon = minTrackCon
        self.mpHands = mp.solutions.hands
        self.mpDraw = mp.solutions.drawing_utils

        self.hands = self.mpHands.Hands(static_image_mode=self.mode,  
                                        max_num_hands=self.maxHands, 
                                        min_detection_confidence=self.detectionCon, 
                                        min_tracking_confidence=self.minTrackCon)
    
    def trackHands(self, frame):
        frame.flags.writeable = False
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

        result = self.hands.process(frame)
        frame.flags.writeable = True
        frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

        if result.multi_hand_landmarks:
            for landmarks in result.multi_hand_landmarks:
                for index, landmark in enumerate(landmarks.landmark):
                    if index == 9:
                        yaw_value = 2*(landmark.x-0.5)
                self.mpDraw.draw_landmarks(frame, landmarks, self.mpHands.HAND_CONNECTIONS)
        else:
            yaw_value = 0

        return yaw_value, frame