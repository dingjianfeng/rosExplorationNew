#! /usr/bin/env python
# coding=utf-8

import numpy as np
import cv2
import sys,getopt

if(len(sys.argv) < 5):
    print 'map_edit.py -i <inputfile> -o <outputfile>'
    print './map_edit.py -i ../map/r1.pgm -o ../map/result1.pgm'
    sys.exit(2)

inputfile = ''
outputfile = ''
try:
    opts, args = getopt.getopt(sys.argv[1:],"hi:o:",["ifile=","ofile="])
except getopt.GetoptError:
    print 'map_edit.py -i <inputfile> -o <outputfile>'
    print './map_edit.py -i ../map/r1.pgm -o ../map/result1.pgm'
    sys.exit(2)

for opt, arg in opts:
    if opt == '-h':
        print 'map_edit.py -i <inputfile> -o <outputfile>'
        sys.exit()
    elif opt in ("-i", "--ifile"):
        inputfile = arg
    elif opt in ("-o", "--ofile"):
        outputfile = arg

print 'Input map file  is :', inputfile
print 'Output map file is : ', outputfile

drawing = False # true if mouse is pressed
mode = True # if True, draw rectangle. Press 'm' to toggle to curve
ix,iy = -1,-1

free = 254
occupied = 0
unknown = 205

color =[(254,254,254),(0,0,0),(205,205,205)]
index = 0
radius = 5

def choose_color(x):
    global index
    index = cv2.getTrackbarPos('color','image')

def choose_radius(x):
    global radius
    radius = cv2.getTrackbarPos('radius','image')

# mouse callback function
def mouse_callback(event,x,y,flags,param):
    global ix,iy,drawing,mode
    global index,radius

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            if mode == True:
                cv2.rectangle(img,(ix,iy),(x,y),color[index],-1)
            else:
                cv2.circle(img,(x,y),radius,color[index],-1)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        if mode == True:
            cv2.rectangle(img,(ix,iy),(x,y),color[index],-1)
        else:
            cv2.circle(img,(x,y),radius,color[index],-1)

img = cv2.imread(inputfile,-1)
cv2.namedWindow('image')
cv2.setMouseCallback('image',mouse_callback)
cv2.createTrackbar('0 : free ,1 :occupied ,2 :unknown , mode change key:m ,exit key: esc ', 'image', 0, 1, choose_color)
cv2.createTrackbar('radius', 'image', 0, 10, choose_radius)
cv2.createTrackbar('color', 'image', 0, 2, choose_color)

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('m'):
        mode = not mode
    elif k == 27 or k == 'q':
        cv2.imwrite(outputfile,img)
        break


cv2.destroyAllWindows()

# print img.shape
#
# cv2.imshow('image',img)
#
# cv2.waitKey(0)
