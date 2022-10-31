import cv2 as cv
import mediapipe as mp

class BodyDetector:
    def __init__(self, mode=False, modelComplexity=0, smooth_lmrks=True, detectionCon=0.8, minTrackCon=0.6):
        self.mode = mode
        self.complexity = modelComplexity
        self.smooth_lmrks = smooth_lmrks
        self.detectionCon = detectionCon
        self.minTrackCon = minTrackCon
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils

        self.pose = self.mpPose.Pose(static_image_mode = self.mode,
                                     model_complexity = self.complexity,
                                     smooth_landmarks = self.smooth_lmrks,
                                     min_detection_confidence = self.detectionCon,
                                     min_tracking_confidence = self.minTrackCon)

    def trackBody(self, frame):
        x, y = [], []
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        result = self.pose.process(frame)
        frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
        if result.pose_landmarks:       #if detected pose, get the points from 0 to 12 and 23 to 24
            landmarks = result.pose_landmarks.landmark
            for lms in landmarks[11:13] + landmarks[23:25]:
                x.append(lms.x)
                y.append(lms.y)
            x_min, x_max = max(x), min(x)
            y_max, y_min = max(y), min(y)
            cv.rectangle(frame, (int(x_min*640), int(y_min*480)),
                                (int(x_max*640), int(y_max*480)), (0, 255, 0), 2)

            #Draw a Circle representing center of the body
            center = round((x_max + x_min)/2, 3), round((y_max + y_min)/2, 3) #Blue
            cv.circle(frame, (int(center[0]*640), int(center[1]*480)), 15, (0, 0, 255), -1)
            self.mpDraw.draw_landmarks(frame, result.pose_landmarks, self.mpPose.POSE_CONNECTIONS)

             #get area:
            area = abs(100*(y_max-y_min))
        else:
            center = (0, 0)
            area = 0
        
        return frame, center, area
    
    def distanceFromCenter(self, frame, centerHuman):
        #Center
        if centerHuman != (0, 0):
            #Draw circle at center of screen
            cv.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), 15, (255, 0, 0), -1)

            #Draw a line connecting the two points
            cv.line(frame, (320, 240), (int(centerHuman[0]*640), int(centerHuman[1]*480)), (255, 255, 0), 10)

            #Find X Y differences           #X                                      #Y
            differences = ((0.5 - centerHuman[0]), (centerHuman[1] - 0.5))
            cv.putText(frame, f"X Delta = {round(differences[0] * 100, 3)}%", (340, 240), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            cv.putText(frame, f"Y Delta = {round(differences[1] * 100, 3)}%", (340, 270), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        else:
            differences = (0, 0)

        return differences

        
