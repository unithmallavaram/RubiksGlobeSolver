import sys
import queue
import heapq
import copy
import gc
from itertools import count

#Enable garbage collection for faster execution times
gc.enable()

#---------------------Execution begins from line 631-------------------


#------------------------- Utility methods -----------------------------

#compare method - returns 1 if equal and 0 if not equal
def compareStates(list1, list2):
    for i in range(len(list1)):
        if list1[i][0] == list2[i][0] and list1[i][1] == list2[i][1]:
            pass
        else:
            return 0
    return 1

#compares two tiles
def compareTiles(list1, list2):
    if list1[0] == list2[0] and list1[1] == list2[1]:
        return 1
    else:
        return 0

#returns the path nodes taking end node and the explored states as input
def getPathNodes(goalNode, explored):
    pathNodes = [goalNode]
    while goalNode["parent"] != "NULL":
        goalNode = explored.get(goalNode["parent"])
        pathNodes.append(goalNode)
    return pathNodes

# There are six available actions in any state; Actions are defined with respect to the front view of the globe
# rightEquator(1), leftEquator(2), upLongitude1(3), downLongitude1(4), ClockwiseLongitude2(5), AntiClockwiseLongitude2
# Transition Model for the globe problem
def transitionModel(parentNode, action):
    # type: (object, object) -> object
    currState = parentNode["state"]
    temp = copy.deepcopy(currState)
    for i in range(len(currState)):
        if action == 1:  # rightEquator
            if currState[i][0] == 90:
                if currState[i][1] == 330:
                    temp[i][1] = 0
                else:
                    temp[i][1] += 30
        elif action == 2:  # leftEquator
            if currState[i][0] == 90:
                if currState[i][1] == 0:
                    temp[i][1] = 330
                else:
                    temp[i][1] -= 30
        elif action == 3:  # upLongitude1
            if currState[i][1] == 0:
                if currState[i][0] == 0:
                    temp[i][0] = 30
                    temp[i][1] = 180
                else:
                    temp[i][0] -= 30
            if currState[i][1] == 180:
                if currState[i][0] == 180:
                    temp[i][0] = 150
                    temp[i][1] = 0
                else:
                    temp[i][0] += 30
        elif action == 4:  # downLongitude1
            if currState[i][1] == 0:
                if currState[i][0] == 150:
                    temp[i][0] = 180
                    temp[i][1] = 180
                else:
                    temp[i][0] += 30
            if currState[i][1] == 180:
                if currState[i][0] == 30:
                    temp[i][0] = 0
                    temp[i][1] = 0
                else:
                    temp[i][0] -= 30
        elif action == 5:  # clockwiseLongitude2
            if currState[i][0] == 0 and currState[i][1] == 0:
                temp[i][0] = 30
                temp[i][1] = 90
            if currState[i][1] == 90:
                if currState[i][0] == 150:
                    temp[i][0] = 180
                    temp[i][1] = 180
                else:
                    temp[i][0] += 30
            if currState[i][0] == 180 and currState[i][1] == 180:
                temp[i][0] = 150
                temp[i][1] = 270
            if currState[i][1] == 270:
                if currState[i][0] == 30:
                    temp[i][0] = 0
                    temp[i][1] = 0
                else:
                    temp[i][0] -= 30
        elif action == 6:  # antiClockwiseLongitude2
            if currState[i][0] == 0 and currState[i][1] == 0:
                temp[i][0] = 30
                temp[i][1] = 270
            if currState[i][1] == 90:
                if currState[i][0] == 30:
                    temp[i][0] = 0
                    temp[i][1] = 0
                else:
                    temp[i][0] -= 30
            if currState[i][0] == 180 and currState[i][1] == 180:
                temp[i][0] = 150
                temp[i][1] = 90
            if currState[i][1] == 270:
                if currState[i][0] == 150:
                    temp[i][0] = 180
                    temp[i][1] = 180
                else:
                    temp[i][0] += 30
    childNode = {"parent": parentNode["id"], "action": action, "state": temp}
    return childNode

#returns the value of heuristic function for a node
def getHeuristic(node):
    #goal state is needed to calculate the distances of each tile from the desired location
    goalState = [[0, 0], [180, 180],
                 [30, 0], [30, 180], [30, 90], [30, 270],
                 [60, 0], [60, 180], [60, 90], [60, 270],
                 [90, 0], [90, 180], [90, 90], [90, 270],
                 [120, 0], [120, 180], [120, 90], [120, 270],
                 [150, 0], [150, 180], [150, 90], [150, 270],
                 [90, 30], [90, 60], [90, 120], [90, 150],
                 [90, 210], [90, 240], [90, 300], [90, 330]]
    currState = node["state"]
    temp = copy.deepcopy(currState)
    distances = []
    i = 0
    for i in range(len(currState)):
        distanceToGoal = getDist(currState[i], goalState[i])
        distances.append(distanceToGoal)
    distances.sort()
    return distances[29]

def getPossibleActions(actions, node):
    if node[1] == 0 or node[1] == 180:
        if node[0] == 0 or node[0] == 180:
            actions.remove(1)
            actions.remove(2)
        elif node[0] == 90:
            actions.remove(5)
            actions.remove(6)
        else:
            actions.remove(1)
            actions.remove(2)
            actions.remove(5)
            actions.remove(6)
    elif node[1] == 90 or node[1] == 270:
        if node[0] == 90:
            actions.remove(3)
            actions.remove(4)
        else:
            actions.remove(1)
            actions.remove(2)
            actions.remove(3)
            actions.remove(4)
    if node[0] == 90 and node[1] != 0 and node[1] != 90 and node[1] != 180 and node[1] != 270:
        actions.remove(3)
        actions.remove(4)
        actions.remove(5)
        actions.remove(6)
    return actions

#get the shortest distance from the target location(implemented using BFS)
def getDist(tile1, tile2):
    if tile1[0] == tile2[0] and tile1[1] == tile2[1]: #both are equal
        return 0
    if tile1[0] == 90 and tile2[0] == 90:
        a = abs((tile1[1]-tile2[1])/30)
        if a > 6:
            return (12-a)
        else:
            return a
    if tile1[1] == 0 and tile2[1] == 0 or tile1[1] == 90 and tile2[1] == 90 or tile1[1] == 180 and tile2[1] == 180 or tile1[1] == 270 and tile2[1] == 270:
        a = (abs(tile1[0]-tile2[0])/30)
        return a

    if tile1[0] == 90 and tile2[0] != 90:
        if tile2[1] == 0:
            a1 = abs((tile1[1]-0)/30)
            if a1>6:
                a1 = 12-a1
        elif tile2[1] == 90:
            a1 = abs((tile1[1] - 90) / 30)
            if a1 > 6:
                a1 = 12 - a1
        elif tile2[1] == 180:
            a1 = abs((tile1[1] - 180) / 30)
            if a1 > 6:
                a1 = 12 - a1
        elif tile2[1] == 270:
            a1 = abs((tile1[1] - 270) / 30)
            if a1 > 6:
                a1 = 12 - a1
        a2 = abs((tile2[0] - 90) / 30)
        if a2 > 6:
            a2 = 12 - a2
        return a1+a2

    if tile2[0] == 90 and tile1[0] != 90:
        if tile1[1] == 0:
            a1 = abs((tile2[1] - 0) / 30)
            if a1 > 6:
                a1 = 12 - a1
        if tile1[1] == 90:
            a1 = abs((tile2[1] - 90) / 30)
            if a1 > 6:
                a1 = 12 - a1
        if tile1[1] == 180:
            a1 = abs((tile2[1] - 180) / 30)
            if a1 > 6:
                a1 = 12 - a1
        if tile1[1] == 270:
            a1 = abs((tile2[1] - 270) / 30)
            if a1 > 6:
                a1 = 12 - a1
        a2 = abs((tile1[0] - 90) / 30)
        if a2 > 6:
            a2 = 12 - a2
        return a1 + a2

    c1 = tile1[1] == 0 and tile2[1] == 180
    c2 = tile1[1] == 180 and tile2[1] == 0
    c3 = tile1[1] == 90 and tile2[1] == 270
    c4 = tile1[1] == 270 and tile2[1] == 90
    c5 = tile1[1] == 0 and tile2[1] == 90
    c6 = tile1[1] == 90 and tile2[1] == 0
    c7 = tile1[1] == 0 and tile2[1] == 270
    c8 = tile1[1] == 270 and tile2[1] == 0
    c9 = tile1[1] == 90 and tile2[1] == 180
    c10 = tile1[1] == 180 and tile2[1] == 90
    c11 = tile1[1] == 180 and tile2[1] == 270
    c12 = tile1[1] == 270 and tile2[1] == 180

    if c1 or c2 or c3 or c4 or c5 or c6 or c7 or c8 or c9 or c10 or c11 or c12:
        #distance from north pole
        a1 = abs(tile1[0]-0)/30
        a2 = abs(tile2[0]-0)/30
        a = a1+a2
        #distance from south pole
        b1 = abs(tile1[0] - 180) / 30
        b2 = abs(tile2[0] - 180) / 30
        b = b1 + b2
        return min(a,b)


#transition model for tile
def transitionModelTile(parentNode, action):
    # type: (object, object) -> object
    currState = parentNode["state"]
    temp = copy.deepcopy(currState)
    if action == 1:  # rightEquator
        if currState[0] == 90:
            if currState[1] == 330:
                temp[1] = 0
            else:
                temp[1] += 30
    elif action == 2:  # leftEquator
        if currState[0] == 90:
            if currState[1] == 0:
                temp[1] = 330
            else:
                temp[1] -= 30
    elif action == 3:  # upLongitude1
        if currState[1] == 0:
            if currState[0] == 0:
                temp[0] = 30
                temp[1] = 180
            else:
                temp[0] -= 30
        if currState[1] == 180:
            if currState[0] == 180:
                temp[0] = 150
                temp[1] = 0
            else:
                temp[0] += 30
    elif action == 4:  # downLongitude1
        if currState[1] == 0:
            if currState[0] == 150:
                temp[0] = 180
                temp[1] = 180
            else:
                temp[0] += 30
        if currState[1] == 180:
            if currState[0] == 30:
                temp[0] = 0
                temp[1] = 0
            else:
                temp[0] -= 30
    elif action == 5:  # clockwiseLongitude2
        if currState[0] == 0 and currState[1] == 0:
            temp[0] = 30
            temp[1] = 90
        if currState[1] == 90:
            if currState[0] == 150:
                temp[0] = 180
                temp[1] = 180
            else:
                temp[0] += 30
        if currState[0] == 180 and currState[1] == 180:
            temp[0] = 150
            temp[1] = 270
        if currState[1] == 270:
            if currState[0] == 30:
                temp[0] = 0
                temp[1] = 0
            else:
                temp[0] -= 30
    elif action == 6:  # antiClockwiseLongitude2
        if currState[0] == 0 and currState[1] == 0:
            temp[0] = 30
            temp[1] = 270
        if currState[1] == 90:
            if currState[0] == 30:
                temp[0] = 0
                temp[1] = 0
            else:
                temp[0] -= 30
        if currState[0] == 180 and currState[1] == 180:
            temp[0] = 150
            temp[1] = 90
        if currState[1] == 270:
            if currState[0] == 150:
                temp[0] = 180
                temp[1] = 180
            else:
                temp[0] += 30
    childNode = {"parent": parentNode["id"], "action": action, "state": temp}
    return childNode

def max(a, b):
    if a > b:
        return a
    else:
        return b

def min(a,b):
    if a < b:
        return a
    else:
        return b

def _sort(list):
    list.sort(key = lambda x: x[0])
    return list

def printSteps(result):
    for i in range(len(result)):
        if result[i] == '1':
            print(" Rotate equator to the right")
        if result[i] == '2':
            print(" Rotate equator to the left")
        if result[i] == '3':
            print(" Rotate 0-180 longitude upwards")
        if result[i] == '4':
            print(" Rotate 0-180 longitude downwards")
        if result[i] == '5':
            print(" Rotate 90-270 longitude clockwise")
        if result[i] == '6':
            print(" Rotate 90-270 longitude anti-clockwise")


#----------------- Definitions of different search algorithms ----------------------

def bfsSearch(initialState, goalState):
    idCounter = 1
    # Root node
    rootNode = {"state": initialState,
                "parent": "NULL",
                "id": idCounter,
                "action": "NULL"}
    idCounter += 1

    # goal test
    goalReached = compareStates(initialState, goalState)
    if goalReached == 1:
        return rootNode

    # frontier
    frontier = queue.Queue(0)
    #put the id of nodes that go into a frontier into a list to enable searching(uses extra space)
    frontierList = []
    frontier.put(rootNode)
    frontierList.append(rootNode["id"])
    queueLength = frontier.qsize()
    #variable to store the max length of the frontier during the process
    maxLength = queueLength
    # explored
    explored = {}
    #BFS search
    while 1:
        if frontier.empty():
            return 'Failure'
        # get the first node in the queue
        node = frontier.get()
        #remove the popped node from the frontier list as well
        explored[node['id']] = node

        actions = [1, 2, 3, 4, 5, 6]
        #remove the action that got the node to this state as doing it again retraces the steps
        if node["action"] != 'NULL':
            if node['action'] == 1:
                actions.remove(2)
            if node['action'] == 2:
                actions.remove(1)
            if node['action'] == 3:
                actions.remove(4)
            if node['action'] == 4:
                actions.remove(3)
            if node['action'] == 5:
                actions.remove(6)
            if node['action'] == 6:
                actions.remove(5)
        for action in actions:
            resNode = transitionModel(node, action)
            resNode["id"] = idCounter
            idCounter += 1
            #check if the resnode is in explored or frontier
            resInExplored = explored.get(str(resNode["id"]))

            if resInExplored != 'None' and (resNode["id"] not in frontierList):   #frontier code to be added
                goalReached = compareStates(resNode["state"], goalState)
                if goalReached == 1:    #solution found
                    #add the number of states and the max size of the queue
                    pathNodes = getPathNodes(resNode, explored)
                    noOfStates = len(explored)
                    pathLength = len(pathNodes) - 1
                    #calculating the path as a sequence of actions
                    j = len(pathNodes) - 2
                    path = ""
                    while j>=0:
                        path += str(pathNodes[j]["action"])
                        j -= 1
                    result = {"noOfStates": noOfStates, "maxQueue": maxLength,"pathLength": pathLength, "path": path}
                    return result
                frontier.put(resNode)
                if maxLength <= frontier.qsize():
                    maxLength = frontier.qsize()

def aStarSearch(initialState, goalState):
    idCounter = 1
    # Root node
    rootNode = {"state": initialState,
                "parent": "NULL",
                "id": idCounter,
                "action": "NULL",
                "g": 0}
    h = getHeuristic(rootNode)
    rootNode["h"] = h
    rootNode["f"] = rootNode["g"] + rootNode["h"]
    idCounter += 1

    # frontier
    frontier = []
    frontierDict = {}
    p = int(rootNode["f"])
    frontier.append([rootNode["f"], rootNode["id"]])
    frontierDict = {1: rootNode}
    queueLength = len(frontier)
    # variable to store the max length of the frontier during the process
    maxLength = queueLength
    # explored
    explored = {}
    # AStar search
    while 1:
        if len(frontier) == 0:
            print("Failure")
            return
        # get the first node in the queue
        element = frontier[0]
        frontier.remove(frontier[0])
        heapq.heapify(frontier)
        node = frontierDict.get(element[1])
        goalReached = compareStates(node["state"], goalState)
        if goalReached == 1:  # solution found
            pathNodes = getPathNodes(node, explored)
            noOfStates = len(explored)
            pathLength = len(pathNodes) - 1
            # calculating the path as a sequence of actions
            j = len(pathNodes) - 2
            path = ""
            while j >= 0:
                path += str(pathNodes[j]["action"])
                j -= 1
            result = {"noOfStates": noOfStates, "maxQueue": maxLength, "pathLength": pathLength, "path": path}
            return result

        explored[node['id']] = node

        actions = [1, 2, 3, 4, 5, 6]
        # remove the action that got the node to this state as doing it again retraces the steps
        if node["action"] != 'NULL':
            if node['action'] == 1:
                actions.remove(2)
            if node['action'] == 2:
                actions.remove(1)
            if node['action'] == 3:
                actions.remove(4)
            if node['action'] == 4:
                actions.remove(3)
            if node['action'] == 5:
                actions.remove(6)
            if node['action'] == 6:
                actions.remove(5)
        for action in actions:
            resNode = transitionModel(node, action)
            resNode["id"] = idCounter
            idCounter += 1
            #path cost of the resultant node
            resNode["g"] = node["g"] + 1
            #heuristic value for the resultant node
            resNode["h"] = getHeuristic(resNode)
            #total value of f
            resNode["f"] = resNode["g"] + resNode["h"]

            # check if the resnode is in explored or frontier
            resInExplored = explored.get(str(resNode["id"]))
            #check if the node is in frontier
            nodeInFrontier = 0
            nodePresent = 0
            for item in frontier:
                if item[1] == resNode:
                    nodeInFrontier = 1
                    nodePresent = item

            if resInExplored != 'None' and not nodeInFrontier:
                p = int(resNode["f"])
                frontier.append([p, resNode["id"]])
                frontierDict[resNode["id"]] = resNode
                heapq.heapify(frontier)
                if maxLength <= len(frontier):
                    maxLength = len(frontier)
            elif nodeInFrontier and nodePresent and nodePresent[0] < resNode["f"]:  #node is in frontier but with higher f value. So replace it
                frontier.remove(nodePresent)
                heapq.heappush(frontier, (resNode["f"], nodePresent[1]))

def rbfs(initialstate, goalState):
    idCounter = 1
    # Root node
    rootNode = {"state": initialState,
                "parent": "NULL",
                "id": idCounter,
                "action": "NULL",
                "g": 0}
    h = getHeuristic(rootNode)
    rootNode["h"] = h
    rootNode["f"] = rootNode["g"] + rootNode["h"]
    idCounter += 1
    explored = {}
    explored[1] = rootNode

    return rbfsSearch(rootNode, goalState, float("inf"), 1, explored)

def rbfsSearch(node, goalState, fLimit, idCounter, explored):
    goalReached = compareStates(node["state"], goalState)
    global exploredStates
    if goalReached == 1:
        result = {}
        result["statesExpanded"] = len(explored)
        temp = copy.deepcopy(node)
        path = ""
        while temp["id"] != 1:
            path = str(temp["action"]) + path
            temp = explored.get(temp["parent"])
        result["path"] = path
        result["pathLength"] = len(path)
        global resultRBFS
        resultRBFS = copy.deepcopy(result)
        return [1, node, node["f"]]
    successors = []
    temp = []
    actions = [1, 2, 3, 4, 5, 6]
    # remove the action that got the node to this state as doing it again retraces the steps
    if node["action"] != 'NULL':
        if node['action'] == 1:
            actions.remove(2)
        if node['action'] == 2:
            actions.remove(1)
        if node['action'] == 3:
            actions.remove(4)
        if node['action'] == 4:
            actions.remove(3)
        if node['action'] == 5:
            actions.remove(6)
        if node['action'] == 6:
            actions.remove(5)
    for action in actions:
        resNode = transitionModel(node, action)
        resNode["id"] = idCounter
        idCounter += 1
        # path cost of the resultant node
        resNode["g"] = node["g"] + 1
        # heuristic value for the resultant node
        resNode["h"] = getHeuristic(resNode)
        # total value of f
        resNode["f"] = resNode["g"] + resNode["h"]
        successors.append([resNode["f"], resNode["id"], resNode])
        temp.append([resNode["f"], resNode["id"]])
        if explored.get(resNode["id"]) == "None":
            exploredStates += 1
    if len(successors) == 0:
        node["f"] = float("inf")
        result = [0, node, node["f"]]
        return result
    for item in successors:
        item[2]["f"] = max(item[2]["f"], node["f"])
        item[0] = item[2]["f"]
    while 1:
        temp.sort()
        for i in successors:
            if i[1] == temp[0][1]:
                bestNode = i[2]
        if bestNode["f"] > fLimit:
            return [0, bestNode, bestNode["f"]]
        for i in successors:
            if i[1] == temp[1][1]:
                alternative = i[2]
        explored[bestNode["id"]] = bestNode
        result = rbfsSearch(bestNode, goalState, min(fLimit, alternative["f"]), idCounter, explored)
        bestNode["f"] = result[2]
        for i in temp:
            if i[1] == bestNode["id"]:
                i[0] = result[2]
        if result[0] == 1:
            return result




# Each state is the list of the current coordinates of the 30 tiles

#------------------ Reading file and executing search --------------------------

# Reading and formatting the input file
inputFile = open(sys.argv[2], "r")
content = inputFile.read()
content = content.splitlines()
del content[0], content[len(content) - 1]
initialState = []
for i in range(len(content)):
    # parse the input to get current lat and long
    # splitting Tile(NP, (30,180), Exact(0,0)) into 'Tile(NP', '(30,180)' and 'Exact(0,0)'. But we need the second string
    splitByComma = content[i].split(', ')
    temp = splitByComma[1].split(',')  # splitting '(30,180)' into '(30' and '180)'
    temp[0] = int(temp[0].replace('(', ''))  # removing ( from '(30'
    temp[1] = int(temp[1].replace(')', ''))  # removing ) from '180)'
    initialState.append(temp)

# goal state is a list with expected latitudes and longitudes
goalState = [[0, 0], [180, 180],
             [30, 0], [30, 180], [30, 90], [30, 270],
             [60, 0], [60, 180], [60, 90], [60, 270],
             [90, 0], [90, 180], [90, 90], [90, 270],
             [120, 0], [120, 180], [120, 90], [120, 270],
             [150, 0], [150, 180], [150, 90], [150, 270],
             [90, 30], [90, 60], [90, 120], [90, 150],
             [90, 210], [90, 240], [90, 300], [90, 330]]

bfsInitialState = copy.deepcopy(initialState)
aStarInitialState = copy.deepcopy(initialState)
rbfsInitialState = copy.deepcopy(initialState)
resultRBFS = {}
exploredStates = 0

#Based on the arguments, different search algorithms are executed

if sys.argv[1] == "BFS":
    solutionBFS = bfsSearch(bfsInitialState, goalState)
    print("BFS for", sys.argv[2])
    print("No of states expanded:", solutionBFS["noOfStates"])
    print("Max size of queue:", solutionBFS["maxQueue"])
    print("No of steps from initial state:", solutionBFS["pathLength"])
    print("Path as a sequence of actions:", solutionBFS["path"])
    printSteps(solutionBFS["path"])

elif sys.argv[1] == "AStar":
    solutionAStar = aStarSearch(aStarInitialState, goalState)
    print("AStar for", sys.argv[2])
    print("No of states expanded:", solutionAStar["noOfStates"])
    print("Max size of queue:", solutionAStar["maxQueue"])
    print("No of steps from initial state:", solutionAStar["pathLength"])
    print("Path as a sequence of actions:", solutionAStar["path"])
    printSteps(solutionAStar["path"])

elif sys.argv[1] == "RBFS":
    solutionRBFS = rbfs(rbfsInitialState, goalState)
    if solutionRBFS[0] == 1:
        print("RBFS for", sys.argv[2])
        print("No of states expanded:", resultRBFS["statesExpanded"]+exploredStates)
        print("No of steps from initial state:", resultRBFS["pathLength"])
        print("Path as a sequence of actions:", resultRBFS["path"])
        printSteps(resultRBFS["path"])


