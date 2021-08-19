# import necessary packages
import cv2
import math


class ObjCenter:
    def __init__(self, haarPath, scale_factor=1.2, minNeighbors=8):
        # load OpenCV's Haar cascade face detector
        self.detector = cv2.CascadeClassifier(haarPath)
        self.last_face_center_x = None
        self.last_face_center_y = None
        self.scale_factor = scale_factor
        self.last_rect = None
        self.minNeighbors = minNeighbors

    def update(self, frame, frameCenter=None):
        # convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # detect all faces in the input frame
        faces = self.detector.detectMultiScale(gray, scaleFactor=self.scale_factor,
                                               minNeighbors=self.minNeighbors, minSize=(30, 30),
                                               flags=cv2.CASCADE_SCALE_IMAGE)

        # check to see if a face was found
        if len(faces) > 0:
            # extract the bounding box coordinates of the face and
            # use the coordinates to determine the center of the
            # face
            # TODO change to finding the face with the largest area
            (x, y, w, h) = faces[0]
            faceX = int(x + (w / 2.0))
            faceY = int(y + (h / 2.0))

            # # attempt to de-jitter the face detect.  Most faces do not move too fast
            # # so see how far the previous face x,y is to the new.  If its to large then
            # # use the previous values
            d = -1
            if self.last_face_center_y and self.last_face_center_x and faceY and faceX:
                d = math.sqrt((faceX - self.last_face_center_x) **
                              2 + (faceY - self.last_face_center_y) ** 2)

            self.last_face_center_x = faceX
            self.last_face_center_y = faceY
            self.last_rect = faces[0]

            # return the center (x, y)-coordinates of the face
            return ((faceX, faceY), faces[0], d)

        # otherwise no faces were found, so return the center of the
        # frame
        if frameCenter:
            return (frameCenter, None, -1)
        else:
            return ((self.last_face_center_x, self.last_face_center_y), None, -1)
