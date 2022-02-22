#!/bin/python3

import glob
import cv2
import os
import numpy as np
import csv
from annot_utils import *
import random

class Annotator:
    def __init__(self):
        self.data_folder = '/home/jgdo/catkin_ws/src/arips_data/datasets/floor/all/'
        self.win = "Frame"

        self.jpgs = glob.glob(self.data_folder + "*.jpg")
        if 0:
            self.jpgs = list(sorted(self.jpgs))
        else:
            random.shuffle(self.jpgs)
        self.image_index = 0

        self.img = None
        self.annotation = None

        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.win, self.onMouse)
        self.reloadFrame()

    def ensureAnnotation(self):
        if self.annotation is None:
            self.annotation = [None, None, 0]

    def getAnnotationStr(self, index):
        if self.annotation[2] == 0:
            return "-1, -1"
        a = self.annotation[index]
        if a is None:
            return "-1, -1"
        else:
            return "{},{}".format(a[0], a[1])

    def saveAnnotation(self):
        if self.annotation is None:
            return

        name = self.jpgs[self.image_index]
        annot_name = getAnnotationName(name)
        with open(annot_name, "w") as file:
            file.write("{},{},{}".format(
                self.getAnnotationStr(0),
                self.getAnnotationStr(1),
                self.annotation[2]
            ))
        print("Saved anotation to {}".format(os.path.basename(annot_name)))

    def reloadFrame(self):
        name = self.jpgs[self.image_index]
        basename = os.path.basename(name)
        #print('Loading frame {}'.format(basename))

        self.img = cv2.imread(name)

        # load annotation
        annot_name = getAnnotationName(name)
        if os.path.isfile(annot_name):
            self.annotation = loadAnnotation(annot_name)
            print("Loaded annotation for {}".format(basename))
        else:
            print("{} has no annotation".format(basename))
            self.annotation = None

        cv2.setWindowTitle(self.win, basename)
        self.show()

    def drawAnnotation(self, img, coords, color):
        if coords is not None:
            cv2.circle(img, (coords[0], coords[1]), 3, color, -1)

    def show(self):
        img = self.img.copy()

        if self.annotation is not None:
            if self.annotation[2] > 0:
                self.drawAnnotation(img, self.annotation[0], (0, 0, 255))
                self.drawAnnotation(img, self.annotation[1], (0, 255, 0))
            else:
                cv2.circle(img, (img.shape[1]//2, img.shape[0]//2), 50, (0, 255, 255), 1)
        cv2.imshow(self.win, img)

    def loadNextUnannotated(self):
        for i in range(self.image_index+1, len(self.jpgs)):
            name = self.jpgs[i]
            annot_name = getAnnotationName(name)
            if not os.path.exists(annot_name):
                print("Found unannotated image: {}".format(os.path.basename(name)))
                self.saveAnnotation()
                self.image_index = i
                self.reloadFrame()
                return

        print("Could not find next unannotated image, consider starting from beginning.")


    def loop(self):
        while True:
            key = cv2.waitKey(50) & 255
            if key == 27:
                break
            elif key == ord('4'):
                if self.image_index > 0:
                    self.saveAnnotation()
                    self.image_index -= 1
                    self.reloadFrame()
            elif key == ord('6'):
                if self.image_index < len(self.jpgs) - 1:
                    self.saveAnnotation()
                    self.image_index += 1
                    self.reloadFrame()
            elif key == ord('x'):
                self.ensureAnnotation()
                self.annotation[2] = 0
                self.show()
            elif key == ord('s'):
                self.saveAnnotation()
            elif key == ord('n'):
                self.loadNextUnannotated()
            elif key != 255:
                print("Key {} '{}' not recognized".format(key, chr(key)))

    def onMouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.ensureAnnotation()
            self.annotation[0] = (x, y)
            self.annotation[2] = 1
            self.show()
        elif event == cv2.EVENT_MBUTTONDOWN:
            self.ensureAnnotation()
            self.annotation[1] = (x, y)
            self.annotation[2] = 1
            self.show()

def main():
    a = Annotator()
    a.loop()

if __name__ == "__main__":
    # execute only if run as a script
    main()
