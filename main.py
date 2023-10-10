import math
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PIL import Image
from matplotlib import animation
import numpy as np
from scipy import interpolate
from scipy.interpolate import CubicSpline
from scipy.interpolate import BSpline
import time
import heapq
import random
from PIL import Image, ImageOps

def scaleToMeters(scalingFactor, distanceInPixels):
    if type(distanceInPixels) == list:
        return [scalingFactor*distanceInPixel for distanceInPixel in distanceInPixels]
    else:
        return scalingFactor * distanceInPixels

def scaleToPixels(scalingFactor, distanceInMeters):
    if type(distanceInMeters)==list:
        return[distanceInMeter/scalingFactor for distanceInMeter in distanceInMeters]
    else:
        distanceInMeters/scalingFactor

def generateEquallySpacedNodes(mapDimensions, spacing):
    horizontalSections = round(mapDimensions[0] / spacing) + 1
    verticalSections = round(mapDimensions[1] / spacing) + 1
    generatedNodesCoordinates = []
    for verticalSection in range(verticalSections):
        coordinates = []
        for horizontalSection in range(horizontalSections):
            coordinates.append((horizontalSection*spacing, verticalSection*spacing))
        if verticalSection % 2 == 1:
            coordinates.reverse()
        generatedNodesCoordinates += coordinates

    return generatedNodesCoordinates

def filterGeneratedNodes(allPixels, image): #filters out the nodes generated on obstacles (black areas)
    filteredNodes = filter(
        lambda point: point[0] < image.size[0] and point[1] < image.size[1] and image.getpixel(point) != 0, allPixels)
    filteredNodes = list(filteredNodes)
    return filteredNodes


def generateNeighbouringGraph(nodeCoordinates, spacing):
    neighbouringGraph = {}
    perpendicularCost = spacing
    diagonalCost = ((2 * spacing ** 2) ** 0.5)
    maxAllowableNeighbourCost = ((2 * spacing ** 2) ** 0.5)

    def addNeighbor(coordinates, cost):
        if coordinates in nodeCoordinates:
            neighbors[coordinates] = cost

    for node in nodeCoordinates:
        neighbors = {}
        possibleneighbours = [
            ((-spacing, spacing), diagonalCost),
            ((-spacing, 0), perpendicularCost),
            ((-spacing, -spacing), diagonalCost),
            ((0, spacing), perpendicularCost),
            ((0, -spacing), perpendicularCost),
            ((spacing, spacing), diagonalCost),
            ((spacing, 0), perpendicularCost),
            ((spacing, -spacing), diagonalCost),
        ]

        for neighbour, cost in possibleneighbours:
            newCoordinates = (node[0] + neighbour[0], node[1] + neighbour[1])
            addNeighbor(newCoordinates, cost)

        if neighbors:
            neighbouringGraph[node] = neighbors
        else:
            neighborsToUpdate = []
            for neighbor in nodeCoordinates:
                x1, y1 = node
                x2, y2 = neighbor
                cost = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
                if 0 < cost <= maxAllowableNeighbourCost:
                    neighborsToUpdate.append(neighbor)
                    neighbors[neighbor] = cost
            neighbouringGraph[node] = neighbors

            for coord1 in neighborsToUpdate:
                neighbors = {}
                neighborsNumber = 12
                for coord2 in nodeCoordinates:
                    if coord1 == coord2:
                        continue
                    if neighborsNumber == 0:
                        break
                    x1, y1 = coord1
                    x2, y2 = coord2
                    cost = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
                    if 0 < cost <= maxAllowableNeighbourCost:
                        neighbors[coord2] = cost
                        neighborsNumber -= 1
                if neighbors:
                    neighbouringGraph[coord1] = neighbors

    return neighbouringGraph


def heuristicsCalculation(nodes, goalNode): #calculating the optimistic cost between nodes in a list and a desired node
    heuristics = {}
    for node in nodes:
        heuristics[node] = ((node[0] - goalNode[0]) ** 2 + (node[1] - goalNode[1]) ** 2) ** 0.5
    return heuristics

def getNeighbours(node, neighboringGraph):
    if node in neighboringGraph:
        return neighboringGraph[node]
    else:
        return None

def AStarAlgorithm(startingNode, goalNode, neighboringGraph):
    heuristics = heuristicsCalculation(coordinates, goalNode)
    openSet = [startingNode] # list to store the nodes to visit
    closedSet = [] # list to store the visited nodes
    g = {}  # list of cost from the starting node to the current node
    parents = {}  # the current parent of the node regarding the last update
    g[startingNode] = 0 # the cost of the starting node = 0
    parents[startingNode] = startingNode # the parent of the starting node is itself
    while len(openSet) > 0:
        n = None # the node with the lowest f= g+h is found
        for temp in openSet:
            if n == None or g[temp] + heuristics[temp] < g[n] + heuristics[n]:
                n = temp
                #should put a break in here to enhance the performance
        if n == goalNode or neighboringGraph[n] == None:
            pass
        else:
            for (m, cost) in getNeighbours(n, neighboringGraph):
                if m not in openSet and m not in closedSet:
                    openSet.append(m)
                    parents[m] = n
                    g[m] = g[n] + cost
                else:
                    if g[m] > g[n] + cost:
                        g[m] = g[n] + cost
                        parents[m] = n
                        if m in closedSet:
                            closedSet.remove(m)
                            openSet.append(m)
        if n == None:
            print('>>>>Path does not exist!<<<<\n')
            return None

        if n == goalNode:
            path = []
            while parents[n] != n:
                path.append(n)
                n = parents[n]
            path.append(startingNode)
            path.reverse()
            totalCost = g[goalNode]
            return [path, totalCost]

        openSet.remove(n)
        closedSet.append(n)
        '''plt.pause(0.001)
        plt.plot(n[0], [rrt.nearestNode.locationY, new[1]], 'go', linestyle="--",
                 markersize=1, linewidth=0.4)'''

    print('>>>>Path does not exist!<<<<\n')
    return None

def getSortedGoalNodes(absoluteStartingNode, goalNodes, neighboringGraph, closedPath = True, nonZiczac = True): #This function resort the goalNodes list so that the optimal order to visit the goal nodes is obtained
    startingNodeHolder = absoluteStartingNode
    goalNodesHolder = goalNodes
    sortedGoalNodesOutput = []

    while len(goalNodesHolder)>0:
        sortedGoalNodes = []
        for node in goalNodesHolder:
            AstarOutput = AStarAlgorithm(startingNodeHolder, node, neighboringGraph)
            #costToAbsoluteStartingNode = AStarAlgorithm(node, absoluteStartingNode, neighboringGraph)
            sortedGoalNodes.append([node, AstarOutput[1]])
        sortedGoalNodes = sorted(sortedGoalNodes, key=lambda x: x[1])

        holder = sortedGoalNodes[0][0]
        if len(sortedGoalNodes) < 3 and closedPath:
            costToAbsoluteStartingNode = AStarAlgorithm(node, absoluteStartingNode, neighboringGraph)
            sortedGoalNodes.append([absoluteStartingNode, costToAbsoluteStartingNode])

        if not nonZiczac:
            if len(sortedGoalNodes) > 1 and startingNodeHolder != absoluteStartingNode:  # for ziczac paths
                costToAbsoluteStartingNode1 = \
                AStarAlgorithm(sortedGoalNodes[0][0], absoluteStartingNode, neighboringGraph)[1]
                costToAbsoluteStartingNode2 = \
                AStarAlgorithm(sortedGoalNodes[1][0], absoluteStartingNode, neighboringGraph)[1]
                if (costToAbsoluteStartingNode1 < costToAbsoluteStartingNode2):
                    holder = sortedGoalNodes[1][0]
        else:
            if len(sortedGoalNodes) > 2:
                costBetweenNode1AndAbsoluteStartingNode = \
                AStarAlgorithm(sortedGoalNodes[0][0], sortedGoalNodes[2][0], neighboringGraph)[1]
                costBetweenNode2AndAbsoluteStartingNode = \
                AStarAlgorithm(sortedGoalNodes[1][0], sortedGoalNodes[2][0], neighboringGraph)[1]
                costBetweenNode1AndNode2 = \
                AStarAlgorithm(sortedGoalNodes[1][0], sortedGoalNodes[0][0], neighboringGraph)[1]
                if sortedGoalNodes[1][1] + costBetweenNode1AndNode2 + costBetweenNode1AndAbsoluteStartingNode < \
                        sortedGoalNodes[0][1] + costBetweenNode1AndNode2 + costBetweenNode2AndAbsoluteStartingNode:
                    holder = sortedGoalNodes[1][0]


        sortedGoalNodesOutput.append(holder)
        startingNodeHolder = holder
        goalNodesHolder.remove(holder)
        print(f"sortedGoalNodesOutput: {sortedGoalNodesOutput}\n")
    if closedPath:
        sortedGoalNodesOutput.append(absoluteStartingNode)
    print("The sorting Process is FINISHED!!\n")
    return sortedGoalNodesOutput


def getOrderedGoalNodes(absoluteStartingNode, goalNodes, neighboringGraph, closedPath = True, nonZiczac = True): #This function resort the goalNodes list so that the optimal order to visit the goal nodes is obtained
    startingNodeHolder = absoluteStartingNode
    goalNodesHolder = goalNodes
    sortedGoalNodesOutput = []

    while len(goalNodesHolder)>0:
        orderedGoalNodes = []

        for node in goalNodesHolder:
            orderedGoalNodes.append([node, math.sqrt((startingNodeHolder[0] - node[0]) ** 2 + (startingNodeHolder[1] - node[1]) ** 2)])
        orderedGoalNodes = sorted(orderedGoalNodes, key=lambda x: x[1])


        if len(orderedGoalNodes)>=3:
            counter = 3
            while counter > 0:
                for node in orderedGoalNodes:
                    orderedGoalNodes[orderedGoalNodes.index(node)][1] = \
                    AStarAlgorithm(startingNodeHolder, node[0], neighboringGraph)[1]
                    counter -= 1
                    if counter == 0:
                        sortedSortedGoalNodes = sorted(orderedGoalNodes, key=lambda x: x[1])
                        if orderedGoalNodes == sortedSortedGoalNodes:
                            break

                        else:
                            orderedGoalNodes = sortedSortedGoalNodes
                            counter = 1# I have to check if putting 1 instead of 3 does not affect the algorithm badly

        holder = orderedGoalNodes[0][0]
        if len(orderedGoalNodes) < 3 and closedPath:
            costToAbsoluteStartingNode = AStarAlgorithm(node, absoluteStartingNode, neighboringGraph)
            orderedGoalNodes.append([absoluteStartingNode, costToAbsoluteStartingNode])

        if not nonZiczac:
            if len(orderedGoalNodes) > 1 and startingNodeHolder != absoluteStartingNode:  # for ziczac paths
                costToAbsoluteStartingNode1 = \
                AStarAlgorithm(orderedGoalNodes[0][0], absoluteStartingNode, neighboringGraph)[1]
                costToAbsoluteStartingNode2 = \
                AStarAlgorithm(orderedGoalNodes[1][0], absoluteStartingNode, neighboringGraph)[1]
                if (costToAbsoluteStartingNode1 < costToAbsoluteStartingNode2):
                    holder = orderedGoalNodes[1][0]
        else:
            if len(orderedGoalNodes) > 2:
                costBetweenNode1AndTemporaryGoalNode = \
                    AStarAlgorithm(orderedGoalNodes[0][0], orderedGoalNodes[2][0], neighboringGraph)[1]
                costBetweenNode2AndTemporaryGoalNode = \
                    AStarAlgorithm(orderedGoalNodes[1][0], orderedGoalNodes[2][0], neighboringGraph)[1]
                costBetweenNode1AndNode2 = \
                    AStarAlgorithm(orderedGoalNodes[0][0], orderedGoalNodes[1][0], neighboringGraph)[1]
                if orderedGoalNodes[0][1] + costBetweenNode1AndNode2 + costBetweenNode2AndTemporaryGoalNode > \
                        orderedGoalNodes[1][1] + costBetweenNode1AndNode2 + costBetweenNode1AndTemporaryGoalNode:
                    holder = orderedGoalNodes[1][0]


        sortedGoalNodesOutput.append(holder)
        startingNodeHolder = holder
        goalNodesHolder.remove(holder)
        print(f"sortedGoalNodesOutput: {sortedGoalNodesOutput}\n")
    if closedPath:
        sortedGoalNodesOutput.append(absoluteStartingNode)
    print("The sorting Process is FINISHED!!\n")
    return sortedGoalNodesOutput


def generateTotalPath(startingNode, orderedGoalNodes, neighboringGraph):
    totalPath = []
    costs = []
    totalCost = 0
    startingNodeHolder = startingNode
    for node in orderedGoalNodes:
        AstarAlgoOutput = AStarAlgorithm(startingNodeHolder, node, neighboringGraph)
        startingNodeHolder = node
        totalPath = totalPath + AstarAlgoOutput[0]
        costs.append(AstarAlgoOutput[1])
        totalCost = totalCost + AstarAlgoOutput[1]
    print("Path Sections Costs in pixels:", costs, "Pixels\n")
    print("Path Sections Costs in meters:", scaleToMeters(scale, costs), "m\n")
    print("Path's Total Cost in pixels:", totalCost, "Pixels\n")
    print("Path's Total Cost in meters:", scaleToMeters(scale, totalCost), "m\n")
    finalTotalPath = []
    for node in totalPath:
        if node != startingNode:
            finalTotalPath.append(node)
    finalTotalPath.insert(0, startingNode)

    print("The total path from the starting node to all goal nodes and then back to the starting node is: \n",
          finalTotalPath, "\n")
    return [finalTotalPath, totalCost]


def getFarOfObstacles(nodeCoordinates, path, spacing):
    modifiedPath = []

    offset_cases = {
        (0, spacing): (0, -spacing),
        (0, -spacing): (0, spacing),
        (spacing, 0): (-spacing, 0),
        (-spacing, 0): (spacing, 0)
    }

    for node in path:
        for offset, replacement_offset in offset_cases.items():
            if (node[0] + offset[0], node[1] + offset[1]) not in nodeCoordinates:
                if (node[0] + replacement_offset[0], node[1] + replacement_offset[1]) in nodeCoordinates:
                    node = (node[0] + replacement_offset[0], node[1] + replacement_offset[1])
        modifiedPath.append(node)

    return modifiedPath


def getInstantaneousCoordinates(instant, timeList, xCoordinates, yCoordinates):
    instant = min(timeList.tolist(), key=lambda x: abs(x - instant))
    x = xCoordinates[timeList.tolist().index(instant)]
    y = yCoordinates[timeList.tolist().index(instant)]
    return (x,y)


def onclick(event):
    global first_point, points
    x, y = event.xdata, event.ydata
    print("Clicked at x = {:.2f}, y = {:.2f}".format(x, y))

    # If this is the first point, store it separately
    if first_point is None:
        first_point = (x, y)
        ax.plot(x, y, 'bo')
        points.append((x,y))
    else:
        # Add the point to the list
        points.append((x, y))
        ax.plot(x, y, 'ro')

    # Update the plot
    fig.canvas.draw()



def clear_points(event):
    global first_point, points
    first_point = None
    points = []
    ax.cla()
    ax.imshow(image)
    fig.canvas.draw()

#################################################################################################################
#################################################################################################################
#################################################################################################################
#################################################################################################################
#################################################################################################################
#################################################################################################################
#################################################################################################################
###################################################   MAIN   ####################################################
#################################################################################################################
#################################################################################################################
#################################################################################################################
#################################################################################################################
#################################################################################################################
#################################################################################################################
#################################################################################################################

imageDirectory = 'map.png'
image = Image.open(imageDirectory)
[imageWidth, imageHeight] = image.size
map = [imageWidth, imageHeight]
print("Map Width: ", imageWidth, " Pixels", "Map Height: ", imageHeight, " Pixels\n")
spacing =40 #pixels
scale= 1 #meters/pixels
constantSpeed = 0.8 #m/s
constantSpeed /=scale #pixel/s


'''
fig, ax = plt.subplots()
plt.title("Select A Starting Node")
ax.imshow(image)
points = []
first_point = None
cid = fig.canvas.mpl_connect('button_press_event', onclick)
fig.canvas.mpl_connect('key_press_event', clear_points)
plt.show()
'''
print("Given Scale: ", scale, " m/pixel\n",  "Given Spacing in Meters: ", scaleToMeters(scale, spacing), " m.\n",\
    "Given Spacing in Pixels: ", spacing, " pixels.\n", \
      "Given Robot Speed: ", scaleToMeters(scale, constantSpeed), "m/s = ", constantSpeed, "pixel/s.\n")
startTime = time.time()
mapCoordinates = generateEquallySpacedNodes(map, spacing)
endTime = time.time()
nodesGenerationTime = endTime-startTime
print("Time to Generate Equally-Spaced Nodes: ", nodesGenerationTime, " s\n")

startTime = time.time()
filteredPoints = filterGeneratedNodes(mapCoordinates, image.convert('L'))
endTime = time.time()
nodesFilteringTime = endTime-startTime
print("Time to Filter Equally-Spaced Nodes: ", nodesFilteringTime, " s\n")

# initializing a list containing the coordinates of user-defined fixed locations in the map
userDefinedNodes = [(2640, 1120), (300, 300), (5266, 1255), (4314, 154), (646, 3678), (2100, 2325), (2710, 106), (1266, 164), (5000, 3660), (3222, 1775), (2475, 3277), (988, 1972), (2645, 2365), (2972, 2383), (2279, 2383), (4000, 2376), (50, 2426), (4313, 2419), (1040, 2680) , (0, 440), (5240, 640), (920, 2400), (2960, 3640), (2320, 3600), (2200, 3160), (360, 3520), (3560, 2160), (560, 0), (2560, 3000), (5080, 120), (1120, 160), (1600, 1160), (3240, 360), (480, 1640), (4800, 3400), (600, 1960), (320, 3480), (880, 760), (2400, 1080), (4120, 1640), (4200, 1400), (2160, 1000), (800, 280), (4400, 600), (160, 1840), (5280, 1880), (4320, 1400), (1840, 520), (2600, 2920), (1880, 3080), (0, 2120), (800, 3040), (1600, 3680), (3880, 1200), (4560, 120), (760, 3560), (5280, 3080), (3560, 560), (2720, 920), (1240, 840), (1880, 2040), (4040, 880), (400, 1360), (2120, 2080), (4400, 3720), (3160, 3760), (3680, 1320), (960, 2440), (4880, 920), (2400, 800), (0, 3360), (2520, 2560), (1160, 1680), (4400, 440), (1800, 360), (3240, 2160), (440, 3640), (5040, 2840), (1560, 3720), (2560, 960), (3160, 2400), (3760, 2160), (3400, 0), (3280, 400), (5120, 680), (3640, 3720), (1680, 1160), (1240, 0), (3280, 640), (520, 1840), (4880, 880), (3400, 40), (1520, 3440), (1120, 920), (2680, 3600), (2280, 3400), (40, 3360), (2880, 680), (5320, 1240), (2520, 960), (3200, 2520), (720, 720), (640, 1160), (4440, 1200), (4960, 0), (3480, 3640), (40, 1400), (1040, 3160), (2520, 3360), (1520, 3280), (2400, 1120), (5200, 1640), (5360, 1640), (5360, 3040), (360, 2880), (560, 2160), (4400, 320), (1080, 520), (2000, 2360)]
#userDefinedNodes = points
print("User-Defined Nodes' coordinates: ", userDefinedNodes, "\n")

coordinates = filteredPoints + userDefinedNodes
print("The number of available nodes: ", len(coordinates), "\n")


[xPoints, yPoints] = zip(*coordinates)
[fixedXPoints, fixedYPoints] = zip(*userDefinedNodes)
fig, ax = plt.subplots()
ax.imshow(image)
plt.title("Path Nodes Generation")
ax.scatter(xPoints, yPoints, s=0.5, c="r", label = "Equally-Spaced Nodes")
ax.scatter(fixedXPoints, fixedYPoints, s=1.5, c="b", label = "User-Defined Nodes")
#ax.legend(loc='lower left')
#plt.xlim(-100, 5500)
plt.tight_layout()
plt.show()

# Generating the neighboring graph
startTime = time.time()
graph = generateNeighbouringGraph(coordinates, spacing)
modifiedGraph = {key: [(inner_key, value) for inner_key, value in value.items()] for key, value in graph.items()}
endTime = time.time()
neighbouringGraphGenerationTime = endTime-startTime
print("Time to Generate Neighbouring Graph: ", neighbouringGraphGenerationTime, " s\n")
#this code transforms our dictionary from a dictionary inside a dictionary format to a list inside a dictionary format
neighboringGraph = modifiedGraph


'''#PyPlot defined nodes
startingNode = points[0] # assigning the absolute starting node
goalNodes = points[1:]'''


#10User-DefinedGoalNodes_IsolatedStartingNode
startingNode = (300, 300) # assigning the absolute starting node
goalNodes = [(5266, 1255), (4314, 154), (646, 3678), (2100, 2325), (2710, 106), (1266, 164), (5000, 3660), (3222, 1775), (2475, 3277), (988, 1972)]# list of goal nodes to visit


'''#10User-DefinedGoalNodes_Non-IsolatedStartingNode
startingNode = (2645, 2365) # assigning the absolute starting node
goalNodes = [(5266, 1255), (4314, 154), (646, 3678), (2100, 2325), (2710, 106), (1266, 164), (5000, 3660), (3222, 1775), (2475, 3277), (988, 1972)]# list of goal nodes to visit
'''
'''##10User-DefinedGoalNodes_Non-IsolatedStartingNodeInZigzaggedDistribution
startingNode = (2645, 2365) # assigning the absolute starting node
goalNodes = [(2645, 2365), (2972, 2383), (2279, 2383), (4000, 2376), (50, 2426), (4313, 2419) ] # list of goal nodes to visit
'''
'''#100RandomlyGeneratedGoalNodes_IsolatedStartingNode
startingNode = (5360, 3040)  # assigning the absolute starting node
startingNode = (300, 300)  # assigning the absolute starting node
goalNodes =  [(1040, 2680), (0, 440), (5240, 640), (920, 2400), (2960, 3640), (2320, 3600), (2200, 3160), (360, 3520), (3560, 2160), (560, 0), (2560, 3000), (5080, 120), (1120, 160), (1600, 1160), (3240, 360), (480, 1640), (4800, 3400), (600, 1960), (320, 3480), (880, 760), (2400, 1080), (4120, 1640), (4200, 1400), (2160, 1000), (800, 280), (4400, 600), (160, 1840), (5280, 1880), (4320, 1400), (1840, 520), (2600, 2920), (1880, 3080), (0, 2120), (800, 3040), (1600, 3680), (3880, 1200), (4560, 120), (760, 3560), (5280, 3080), (3560, 560), (2720, 920), (1240, 840), (1880, 2040), (4040, 880), (400, 1360), (2120, 2080), (4400, 3720), (3160, 3760), (3680, 1320), (960, 2440), (4880, 920), (2400, 800), (0, 3360), (2520, 2560), (1160, 1680), (4400, 440), (1800, 360), (3240, 2160), (440, 3640), (5040, 2840), (1560, 3720), (2560, 960), (3160, 2400), (3760, 2160), (3400, 0), (3280, 400), (5120, 680), (3640, 3720), (1680, 1160), (1240, 0), (3280, 640), (520, 1840), (4880, 880), (3400, 40), (1520, 3440), (1120, 920), (2680, 3600), (2280, 3400), (40, 3360), (2880, 680), (5320, 1240), (2520, 960), (3200, 2520), (720, 720), (640, 1160), (4440, 1200), (4960, 0), (3480, 3640), (40, 1400), (1040, 3160), (2520, 3360), (1520, 3280), (2400, 1120), (5200, 1640), (5360, 1640), (360, 2880), (560, 2160), (4400, 320), (1080, 520), (2000, 2360)]
'''
'''##10User-DefinedGoalNodes_Non-IsolatedStartingNodeInZigzaggedDistribution
startingNode = (2645, 2365) # assigning the absolute starting node
goalNodes = [(2645, 2365), (2972, 2383), (2279, 2383), (4000, 2376), (50, 2426), (4313, 2419) ] # list of goal nodes to visit'''

'''#100RandomlyGeneratedGoalNodes_Non-IsolatedStartingNode
startingNode = (3680, 1320)  # assigning the absolute starting node
goalNodes = [(5360, 3040), (1040, 2680), (0, 440), (5240, 640), (920, 2400), (2960, 3640), (2320, 3600), (2200, 3160), (360, 3520), (3560, 2160), (560, 0), (2560, 3000), (5080, 120), (1120, 160), (1600, 1160), (3240, 360), (480, 1640), (4800, 3400), (600, 1960), (320, 3480), (880, 760), (2400, 1080), (4120, 1640), (4200, 1400), (2160, 1000), (800, 280), (4400, 600), (160, 1840), (5280, 1880), (4320, 1400), (1840, 520), (2600, 2920), (1880, 3080), (0, 2120), (800, 3040), (1600, 3680), (3880, 1200), (4560, 120), (760, 3560), (5280, 3080), (3560, 560), (2720, 920), (1240, 840), (1880, 2040), (4040, 880), (400, 1360), (2120, 2080), (4400, 3720), (3160, 3760), (960, 2440), (4880, 920), (2400, 800), (0, 3360), (2520, 2560), (1160, 1680), (4400, 440), (1800, 360), (3240, 2160), (440, 3640), (5040, 2840), (1560, 3720), (2560, 960), (3160, 2400), (3760, 2160), (3400, 0), (3280, 400), (5120, 680), (3640, 3720), (1680, 1160), (1240, 0), (3280, 640), (520, 1840), (4880, 880), (3400, 40), (1520, 3440), (1120, 920), (2680, 3600), (2280, 3400), (40, 3360), (2880, 680), (5320, 1240), (2520, 960), (3200, 2520), (720, 720), (640, 1160), (4440, 1200), (4960, 0), (3480, 3640), (40, 1400), (1040, 3160), (2520, 3360), (1520, 3280), (2400, 1120), (5200, 1640), (5360, 1640), (360, 2880), (560, 2160), (4400, 320), (1080, 520), (2000, 2360)] 
'''




print("Starting Node: ", startingNode, "\nGoal Nodes: ", goalNodes, "\n")
xGoals, yGoals = zip(*goalNodes)
fig, ax = plt.subplots()
ax.imshow(image)
ax.scatter(xGoals, yGoals, s= 30, c = 'g', label = 'Goal Nodes')
ax.scatter(startingNode[0], startingNode[1], s = 40, c = 'm', label = 'Starting Node')
plt.title("Starting & Goal Nodes")
#plt.xlim(-100, 5500)
plt.tight_layout()
#ax.legend(loc='best')
plt.show()




startTime = time.time()
orderedGoalNodes = getOrderedGoalNodes(startingNode, goalNodes, neighboringGraph, True, True)
endTime = time.time()
goalNodesSortingTime = endTime-startTime
print("Time to Order Goal Nodes: ", goalNodesSortingTime, " s\n")


startTime = time.time()
totalPathGeneration = generateTotalPath(startingNode, orderedGoalNodes, neighboringGraph)
finalTotalPath = totalPathGeneration[0]
#finalTotalPath = getFarOfObstacles(coordinates, finalTotalPath, spacing)
endTime = time.time()
totalPathGenerationTime = endTime-startTime
print("Time to Generate Total Path: ", totalPathGenerationTime, " s\n")
[x, y] = zip(*finalTotalPath)
fig, ax = plt.subplots()
ax.imshow(image)
ax.plot(x, y, c='gold', label="Generated Paths")
ax.scatter(x, y, s=3, c="r", label = 'Path Nodes')
ax.scatter(xGoals, yGoals, s= 30, c = 'g', label = 'Goal Nodes')
ax.scatter(startingNode[0], startingNode[1], s = 40, c = 'm', label = 'Starting Node')
'''circle = plt.Circle((x[0], y[0]), radius=40, color='b', label = 'The Robot')
ax.add_artist(circle)

def animate(i):
    circle.center = (x[i], y[i])
    return circle,

ani = animation.FuncAnimation(fig, animate, frames=len(x), interval=60)
ani.save("animation.gif", writer='pillow')'''
plt.title("Overall Path")
#plt.xlim(-100, 5500)
plt.tight_layout()
#ax.legend(loc='best')
plt.show()

totalCost = totalPathGeneration[1]
time = math.ceil(totalCost/constantSpeed)
print(f"Needed Time to Complete The Path: {time} s\n")
t = np.linspace(0, time, len(finalTotalPath))
#print(t, "\n")
path = np.array(finalTotalPath)
x = path[:, 0]
y = path[:, 1]

cs1 = CubicSpline(t, x)
cs2 = CubicSpline(t, y)

t_new = np.linspace(0, time, num=400)
x_new = cs1(t_new)
y_new = cs2(t_new)

fig, ax = plt.subplots()
ax.imshow(image)
ax.plot(x_new, y_new, c='gold', label='x-y Smooth Cubic spline')
ax.scatter(x, y, s=5, c="r", label = 'Path Nodes')
ax.scatter(xGoals, yGoals, s= 15, c = 'g', label = 'Goal Nodes')
ax.scatter(startingNode[0], startingNode[1], s = 20, c = 'm', label = 'Starting Node')
plt.title("Path Smoothing and Interpolation Using Cubic Splines")
#plt.xlim(-100, 5500)
plt.tight_layout()
plt.show()
cubicSplinesPathNodes = []
for (x, y) in zip(x_new, y_new):
    cubicSplinesPathNodes.append((x, y))

print("Cubic Splines Smoothed Path:\n\n\n\n\n\n", cubicSplinesPathNodes)
print("The coordinates of the robot at 2s is:", getInstantaneousCoordinates(2, t_new, x_new, y_new), "\n")