# -*- coding: utf-8 -*-

import cv2
import numpy as np
from bisect import insort

#Color definitions for the map
COLOR_EMPTY =		[255,	255,	255]
COLOR_INITIAL =		[0,		0,		255]
COLOR_DESTINATION =	[255,	0,		0  ]
COLOR_WALL =		[0,		255,	0  ]
COLOR_PATH = 		[0,		0,		0  ]
dd = np.sqrt(2) #Diagonal distance

class Node:
	def __init__(self, point, best_parent, cost, heuristic=None):
		self.point = point
		self.best_parent = best_parent
		self.cost = cost
		self.heuristic = heuristic

	def total_cost(self):
		assert(self.heuristic is not None)
		return self.cost + self.heuristic

	def __lt__(self, other):
		return self.total_cost() < other.total_cost()

	def __eq__(self, other):
		return self.point == other.point


class Map:
	"""Map where the path will be searched"""

	def __init__(self, dims, init_point, dest_point):
		height, width = dims
		assert(len(init_point) == 2 and len(dest_point) == 2)
		assert(init_point[0] < height and init_point[1] < width)
		assert(dest_point[0] < height and dest_point[1] < width)
		assert(init_point != dest_point)

		self.init_point = init_point
		self.dest_point = dest_point

		self.dims = dims

		self.obstacles = np.full((height,width), False, dtype=np.bool)

#Info about heuristics:
#http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html

#Calculates the heuristic for the node using the
#euclidian distance to the destination point
def calculateHeuristic_ed(node, map):
	node.heuristic = euclidianDistance(node.point, map.dest_point)

#Calculates the heuristic for the node using the
#octile distance to the destination point
def calculateHeuristic_od(node, map):
	dr = abs(node.point[0] - map.dest_point[0]) #Distance in rows
	dc = abs(node.point[1] - map.dest_point[1]) #Distance in columns

	node.heuristic = (dr + dc) + (dd - 2) * min(dr, dc)

def euclidianDistance(pA, pB):
	return np.sqrt((pA[0]-pB[0])**2 + (pA[1]-pB[1])**2)

#Generates the children of the node in the map as a list of Nodes
def generateChildren(node, map):
	rows, cols = map.dims
	children = []

	for i in xrange(-1,2):
		for j in xrange(-1,2):
			row, col = node.point[0]+i, node.point[1]+j

			if (not (i==0 and j==0) and				#Not the same point
				map.obstacles[row, col] == False and #Not an obstacle point
				(row >= 0) and (row < rows) and	#Inside limits of map
				(col >= 0) and (col < cols)):

				#If it's on a diagonal, the increment in cost is sqrt(2)
				#If not it's 1
				d_cost = dd #Diagonal distance
				if (i==0) or (j==0):
					d_cost = 1.0

				child = Node((row, col), node.point, node.cost + d_cost)
				calculateHeuristic_od(child, map)
				children.append(child)

	return children

#Makes an A* search over the map and obtains the path from
#the initial point to the destination point avoiding obstacles.
#Returns the list of points in the path
def aStarSearch(map):
	initial_node = Node(map.init_point, None, 0)

	open_set = [initial_node]
	closed_set = []

	found = False
	while open_set:
		current = open_set.pop(0) #Take the element with the lowest cost (open_set remains ordered)
		closed_set.append(current)

		if current.point == map.dest_point:
			found = True
			break

		children = generateChildren(current, map)
		for child in children:
			if child in closed_set: #Child already in closed_set (update info)
				updateNodeInfo(child, closed_set)

			elif child in open_set:
				idx = open_set.index(child)
				if open_set[idx].cost > child.cost:
					open_set[idx] = child

			else:
				insort(open_set, child) #Add in order of TOTAL cost

	if not found: #Path does not exist
		return []

	else:
		dest = closed_set[-1]  #Destination will be the last node inserted into closed_set
		path = [dest.point]
		next_point = dest.best_parent
		while next_point is not None:
			for i in xrange(len(closed_set)): #Search for best parent
				if closed_set[i].point == next_point:
					path.append(closed_set[i].point)
					next_point = closed_set[i].best_parent
					break

		return path

#Updates the node and its descendants' info recursively
def updateNodeInfo(node, closed_set):
	idx = closed_set.index(node)

	if closed_set[idx].cost > node.cost:
		#It's a better candidate
		closed_set[idx] = node

		for i in xrange(len(closed_set)): #For every descendant
			if closed_set[i].best_parent == node.point:
				desc = closed_set[i]
				#Distance to parent
				dist = dd #Diagonal distance
				if desc.point[0] == node.point[0] or desc.point[1] == node.point[1]:
					dist = 1.0

				new = Node(desc.point,
						   node.point,
						   node.cost + dist,
						   desc.heuristic)

				updateNodeInfo(new, closed_set)


def createMapRepresentation(map, path=None):
	init_point = map.init_point
	dest_point = map.dest_point
	height, width = map.dims

	#Blank map
	representation = np.full((height,width,3), 255, dtype=np.uint8)
	tam = 3

	#Add obstacles
	for i in xrange(height):
		for j in xrange(width):
			if map.obstacles.item((i,j)):
				representation.itemset((i,j,0), COLOR_WALL[0])
				representation.itemset((i,j,1), COLOR_WALL[1])
				representation.itemset((i,j,2), COLOR_WALL[2])

	#Add initial point
	row, col = init_point
	representation[row-tam:row+tam+1, col-tam:col+tam+1] = COLOR_INITIAL

	#Add destination point
	row, col = dest_point
	representation[row-tam:row+tam+1, col-tam:col+tam+1] = COLOR_DESTINATION

	if path is not None:
		for point in path:
			representation[point[0], point[1]] = COLOR_PATH

	return representation
