# coding: utf-8

__author__ = "Sérgio José de Sousa"

import numpy as np
from heapq import heappush, heappop
from math import sqrt
import sys
import time
sys.setrecursionlimit(5000)

# Save a file with de maze and de path
printMaze = True
# Print the time of execution
printTime = False

# Calculate the heuristic between two coordinates
# hType = heuristic name (valids = manhattan, octile, euclidean)
# vStart = start coordenate
# vGoal = goal coordenate
# return value of heuristic
def Heuristic(hType, vStart, vGoal):
    if hType == 'manhattan':
        dx = abs(vStart[0] - vGoal[0])
        dy = abs(vStart[1] - vGoal[1])
        return (dx + dy)
        
    elif hType == 'octile':
        dx = abs(vStart[0] - vGoal[0])
        dy = abs(vStart[1] - vGoal[1])
        return max(dx, dy) + 0.5 * min(dx, dy)
    elif hType == 'euclidean':
        dx = abs(vStart[0] - vGoal[0])**2
        dy = abs(vStart[1] - vGoal[1])**2
        return sqrt(dx + dy)
    
# Convert maze matrix, '@' to wall and '.' to free way, into adjacency list
# maze = matrix n x m
# width = width of the matrix
# height = height of the matrix
# withDiagonal = boolean to assign diagonal paths
# cN, cS, cE, cW, cNE, cNW, cSE, cSW = cost of the arrows
# return graph of adjacency list
def maze2graph(maze, width, height, withDiagonal, cN = 1., cS = 1., cE = 1., cW = 1., cNE = 1.5, cNW = 1.5, cSE = 1.5, cSW = 1.5):
    
    #Generate a empty dictionary with the valides vertices
    graph = {}
    for j in range(width):
        for i in range(height):
             if maze[i][j] == '.':
                    graph[(i, j)] = []
        
    #Add adjacency listo to each vertice
    for row, col in graph.keys():

        if (row < height - 1 and maze[row + 1][col] == '.'):
            graph[(row, col)].append(((row + 1, col), cS))
            graph[(row + 1, col)].append(((row, col), cN))
                     
        if (col < width - 1 and maze[row][col + 1] == '.'):            
            graph[(row, col)].append(((row, col + 1), cE))
            graph[(row, col + 1)].append(((row, col), cW))
            
        if withDiagonal:
            if (col < width - 1 and row - 1 >= 0 and maze[row - 1][col + 1] == '.'):
                if ( maze[row - 1][col] == '.' and maze[row][col + 1] == '.'): 
                    graph[(row, col)].append(((row - 1, col + 1), cNE))            
                    graph[(row - 1, col + 1)].append(((row, col), cSW))

            if (col < width - 1 and row < height - 1 and maze[row + 1][col + 1] == '.'):
                if ( maze[row + 1][col] == '.' and maze[row][col + 1] == '.'):            
                    graph[(row, col)].append(((row + 1, col + 1), cSE))            
                    graph[(row + 1, col + 1)].append(((row, col), cNW))
                      
    return graph

# Main method to search in adjacency list
# graph = adjacency list in dictionary format
# aType = type of search algorithm
# start = start node
# goal = goal node
# limiteDepth = limite of Depth Limited Search
# heuristic = heuristic type
# return boolean if find path and the list of visited nodes
def GenericSearch(graph, aType, start, goal, limiteDepth = -1, heuristic = ''):
    # Constants positions of vector of OpenSet
    Cost_cOS = 0 # Cost of node
    Node_cOS = 1 # node
      
    # Constants positions of visited nodes
    Status_Vi = 0 # -1 = seen but not added openset, 0 = added openset, 1 = closed node
    Father_Vi = 1 # node of father with actual best path
    Cost_Vi = 2 # Cost until the node
    Depth_Vi = 3 # Depth until the node
    
    # Initialize each list, visited and openset list
    if aType == 'AStar' or aType == 'BG':
        visited = {start : [0, None, Heuristic(heuristic, start ,goal), 0]}
        openSet = [(Heuristic(heuristic, start ,goal), start)]
    else:
        visited = {start : [0, None, 0, 0]}
        openSet = [(0, start)]
    
    # DLS = Depth Limited Search, IDS = Iterative Deepining Search, DFS = Deep First Search
    if aType == 'DLS' or aType == 'DFS':
        ordenation = 'LIFO'
    # BFS = Breadth First Search    
    elif aType == 'BFS':
        ordenation = 'FIFO'
    # AStar = A*, BG = Best First Search, UCS = Uniform Cost Search    
    elif aType == 'AStar' or aType == 'BG' or aType == 'UCS':
        ordenation = 'extractMin'

    while openSet:
        # Extract node in openset
        if ordenation == 'FIFO':
            node = openSet.pop(0)            
        elif ordenation == 'LIFO' or aType == 'DLS':
            node = openSet.pop()
        elif ordenation == 'extractMin':
            node = heappop(openSet)
        # Verify if this node es goal    
        if node[Node_cOS] == goal:
            return True, visited
        
        # Cost until this node
        costFather = visited[node[Node_cOS]][Cost_Vi]
        # Depth until this node
        depthFather = visited[node[Node_cOS]][Depth_Vi]
        
        # In IDS, verify if the node is in the edge of limite
        if aType == 'DLS' and costFather >= limiteDepth:
            visited[node[Node_cOS]][Status_Vi] = 1
            continue
        # Scroll through of adjacency list
        for adj, cost in graph[node[Node_cOS]]:
            # If this adjacency node is not in visited
            if not visited.has_key(adj):
                visited[adj] = [-1, 
                                None, 
                                float('inf'),
                                depthFather+cost
                               ]
            # To DLS methods
            if aType == 'DLS':
                # Verify if the new cost is equal or less the limite depth
                if (costFather + cost) <= limiteDepth:
                    #  Verify if the new cost is less the old
                    if visited[adj][Cost_Vi] > (costFather + cost):
                        # If the node is not in the openset, it is added
                        if visited[adj][Status_Vi] != 0:                         
                            openSet.append((cost, adj))
                        # update node with the new cost and new depth
                        visited[adj] = [0, 
                                        node[Node_cOS], 
                                        (costFather + cost),
                                        depthFather+cost]
                # otherwise, close the node
                else:
                    visited[adj][Status_Vi] = 1 
                                    
            else:
                # to other methods, verify if the node is not closed
                if visited[adj][Status_Vi] != 1:
                    # to methods with use extraction of the minimum value node
                    if ordenation == 'extractMin':
                        # if A*
                        if aType == 'AStar':
                            # Calc of f(n) = g(n)+h(n)
                            costHeuristic = depthFather + cost + Heuristic(heuristic, adj ,goal)
                            
                            if visited[adj][Cost_Vi] > costHeuristic:                        
                                visited[adj] = [0, 
                                                node[Node_cOS], 
                                                costHeuristic,
                                                depthFather+cost]
                                # push to ordened list
                                heappush(openSet, (costHeuristic, adj))
                        # if Best First Search        
                        elif aType == 'BG':
                            # Calc of f(n) = h(n)
                            costHeuristic = Heuristic(heuristic, adj ,goal)
                            
                            if visited[adj][Cost_Vi] > costHeuristic:                        
                                visited[adj] = [0, 
                                                node[Node_cOS], 
                                                costHeuristic,
                                                depthFather+cost]

                                heappush(openSet, (costHeuristic, adj))
                        # Else UCS        
                        else:
                            if visited[adj][Cost_Vi] > (costFather + cost): 
                                # Calc of f(n) = g(n)                     
                                visited[adj] = [0, 
                                                node[Node_cOS], 
                                                (costFather + cost),
                                                depthFather+cost]
                                # push to ordened list
                                heappush(openSet, (costFather + cost, adj))

                    # to methods with use normal extraction        
                    elif ordenation == 'FIFO' or ordenation == 'LIFO':
                        if visited[adj][Status_Vi] == -1:
                            visited[adj] = [0, 
                                            node[Node_cOS], 
                                            (costFather + cost),
                                            depthFather+cost]
                            # push to ordened list
                            openSet.append((cost, adj))

                                                    
        # Close de node        
        visited[node[Node_cOS]][Status_Vi] = 1
    # If the goal is not accessible            
    return False, visited

# IDS = Iterative Deepining Search
# graph = adjacency list in dictionary format
# start = start node
# goal = goal node
# maxDepth = custom limit
# return boolean if find path and the list of visited nodes
def IDS(graph, start, goal, maxDepth = 0):
    if maxDepth == 0:
        maxDepth = mapH*mapW
    print mapH

    for limteDepth in range(1, maxDepth+1):
        msg, visited = GenericSearch(graph, 'DLS', start, goal, limteDepth)

        if msg:
            return msg, visited
    
    return msg, visited

# Print the path on file maze
# goal = goal node
# closeSet = list of visited nodes
# fileName = file name to save the maze
# updatePath = print the short path
def PrintMap(goal, closedSet, fileName='fileName.txt', updatePath=True):
    mapClone = mMap.copy()
    count = 0
    if updatePath:
        UpdateMap(goal, 0, closedSet, mapClone)        
    
    for x, y in closedSet:
        if mapClone[x][y] != 'X':
            if closedSet[(x,y)][0] == 1:
                mapClone[x][y] = 'o'
            else:
                mapClone[x][y] = ':'
    
    thefile = open(fileName, 'w')

    for item in mapClone:
          thefile.writelines(''.join(item)+'\n')

# Method used to print short way founded in maze matrix
# goal = goal node
# count = count iterations
# closeSet = list of visited nodes
# mapClone = clone of the maze matrix
def UpdateMap(goal, count, closedSet, mapClone):
    mapClone[goal[0]][goal[1]] = 'X'
    if closedSet[goal][1] != None:
        count += 1
        UpdateMap(closedSet[goal][1], count, closedSet, mapClone)
    return
            
# Print the path on terminal
# vStart = start node
# vGoal = goal node
# closeSet = list of visited nodes
def Output(vStart, vGoal, closedSet):
    outPutList = []
    path = ListGenerate(vGoal, closedSet, outPutList)
    
    print '<' + str(path[0][0][0]) + ', ' + str(path[0][0][1]) + ', ' + str(path[0][1])+'>'
    print '<' + str(path[-1][0][0]) + ', ' + str(path[-1][0][1]) + ', ' + str(path[-1][1])+'>'
    print ''
    concat = ""
    for node in path:
        concat += '<' + str(node[0][0]) + ', ' + str(node[0][1]) + ', ' + str(node[1])+'> '
    print concat.strip()

# Methode used to generate list with the path
# goal = goal node
# closeSet = list of visited nodes
# outPutList = list with the path
# return the list
def ListGenerate(goal, closedSet, outPutList = []):
    if closedSet[goal][1] != None:
        ListGenerate(closedSet[goal][1], closedSet, outPutList)
        outPutList.append([goal, closedSet[goal][3]])
        return outPutList
    
    outPutList.append([goal, closedSet[goal][3]]) 
    return outPutList

# Print output to invalid path
# vStart = start node
# vGoal = goal node
def OutputNotPath(vStart, vGoal):
    print '<' + str(vStart[0]) + ', ' + str(vStart[1]) + ', ' + '0>'
    print '<' + str(vGoal[0]) + ', ' + str(vGoal[1]) + ', ' + 'inf>'

# Initializate process
def Start():
    file = open(vMap, "r").read().splitlines() 
    '''
    1 a linha contém o tipo do mapa
    2 a e 3 a linhas correspondem à altura e largura respectivamente,
    4 a linha contém o nome do mapa
    '''

    global mapType
    global mapH
    global mapW
    global mapName
    global mMap

    line = 1
    for l in file:
        if line == 1:
            mapType = l
        elif line == 2:
            mapH = int(l.split(" ")[1])        
        elif line == 3:
            mapW = int(l.split(" ")[1])        
            mMap = np.empty((0, mapW))
        elif line == 4:
            mapName = l
        else:
            mLine = np.array(list(l))
            mMap = np.append(mMap, [mLine], axis=0)
        line += 1

    #Generate the graph    
    graph = maze2graph(mMap, mapW, mapH, True)

    if not graph.has_key(vStart) or not graph.has_key(vGoal):
        OutputNotPath(vStart, vGoal)
    else:
        start = time.time()
        msg = False

        if alg == 'astar':
            msg, vis = GenericSearch(graph, 'AStar',vStart, vGoal, heuristic=heuristic)
        elif alg == 'ucs':
            msg, vis = GenericSearch(graph, 'UCS',vStart, vGoal)
        elif alg == 'bg':
            msg, vis = GenericSearch(graph, 'BG',vStart, vGoal, heuristic=heuristic)
        elif alg == 'ids':
            testMsg, testVis = GenericSearch(graph, 'AStar',vStart, vGoal, heuristic='manhattan')
            if testMsg:
                msg, vis = IDS(graph, vStart, vGoal)
            else:
                vis = None
                msg = False
        elif alg == 'dls':
            msg, vis = GenericSearch(graph, 'DLS',vStart, vGoal, limiteDepth=vDepth)
        elif alg == 'bfs':
            msg, vis = GenericSearch(graph, 'BFS',vStart, vGoal)
        elif alg == 'dfs':
            msg, vis = GenericSearch(graph, 'DFS',vStart, vGoal)
        
        if msg:
            Output(vStart, vGoal, vis)

            if printMaze:
                PrintMap(vGoal, vis, vMap+'.'+alg+'.txt')

        else:
            OutputNotPath(vStart, vGoal)
        end = time.time()
        
        if printTime:
            print 'Time:',(end - start)/60


######################
###Global Variables###
######################
alg = ''
vMap = ''
vStart = None
vGoal = None
heuristic = ''
vDepth = 0

mapType = ''
mapName = ''
mapH = 0
mapW = 0
mMap = None
graph = {}

#Take the args and initializate variables
try:
    if len(sys.argv) >= 7:
        alg = sys.argv[1]
        vMap = sys.argv[2]
        vStart = (int(sys.argv[3]), int(sys.argv[4]))
        vGoal = (int(sys.argv[5]), int(sys.argv[6]))
        #(1 = distance manhattan, 2 = distance octile, 3 = euclidean)
        if (alg == 'bg'):
            heuristic = 'manhattan'
        if (alg == 'astar'):
            if len(sys.argv) >= 8:
                if int(sys.argv[7]) == 1:
                    heuristic = 'manhattan'
                elif int(sys.argv[7]) == 2:
                    heuristic = 'octile'
                elif int(sys.argv[7]) == 3:
                    heuristic = 'euclidean'
                else:
                    sys.exit("Error on args")

            else:
                sys.exit("Error, args is missing!!")
    else:
        sys.exit("Error, args is missing!!")

except Exception as e:
    sys.exit("Error on args")

#Start the search
Start()