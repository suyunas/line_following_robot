#!/usr/bin/env python
# Define the Yellow Colour in HSV
#RGB
#[[[222,255,0]]]
#BGR
#[[[0,255,222]]]

import numpy as np
import cv2

yellow = np.uint8([[[130,0,75]]])
hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
print (hsv_yellow)
#[[[ 34 255 255]