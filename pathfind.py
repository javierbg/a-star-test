#!/usr/bin/env python

from astar import *

import cv2

drawing = False
obstacles = None
recalculate = False

####################################################
def draw(event, x, y, flags, param):
	global drawing, obstacles, recalculate
	tam = 3

	if event == cv2.EVENT_LBUTTONDOWN:
		drawing = True

	elif event == cv2.EVENT_MOUSEMOVE:
		if drawing:
			if x < img.shape[1] and y < img.shape[0]:
				obstacles[y-tam:y+tam, x-tam :x+tam] = True

	elif event == cv2.EVENT_LBUTTONUP:
		drawing = False
		recalculate = True

####################################################

#PARAMETERS
size = (200, 200)
initial_point = (10,10)
destination_point = (100, 100)

window_name = 'Map'
m = Map(size, initial_point, destination_point)
obstacles = m.obstacles

cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback(window_name, draw)

path = aStarSearch(m)
while True:
	if recalculate:
		path = aStarSearch(m)
		recalculate = False

	img = createMapRepresentation(m, path)
	cv2.imshow(window_name, img)
	cv2.waitKey(4)
