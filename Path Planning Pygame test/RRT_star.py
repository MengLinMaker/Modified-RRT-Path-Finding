from CoordinateTree import CoordinateTree
import random as rnd
import math
import sys

sys.setrecursionlimit(10000)
print(sys.getrecursionlimit())
#threading.stack_size(0x2000000)

# RRT is a subclass of CoordinateTree
class RRT_star(CoordinateTree):
    def __init__(self, mapDim, minCoor = [0,0], obsList = []):
        # inherit methods and values from
        super().__init__([0,0])
        self.goal = [0,0]
        self.goalID = []
        self.goalDist2root = []
        self.path = []

        self.obsList = obsList

        self.mapDim = mapDim
        self.minCoor = minCoor

        self.interpPoints = 1
        self.maxStep = 10
        self.neighbourSize = 5*self.maxStep
        self.densityDist = self.maxStep
        self.toGoalStep = self.neighbourSize/2
        self.shortenIter = 100
        self.backtrackSteps = 5

        self.counter = 0

        self.ellipseMatrix = None
        self.sampleLength = 10

    def reset(self, root):
        self.root = root
        self.x = [root[0]]
        self.y = [root[1]]
        self.parentIDs = [0]
        self.childIDs = [[]]
        self.dist2root = [0]
        self.numNodes = 1
        self.goalID = []
        self.goalDist2root = []
        self.path = []
        self.ellipseMatrix = None

    def removeAllNodes(self, root):
        self.root = root
        self.x = [root[0]]
        self.y = [root[1]]
        self.parentIDs = [0]
        self.dist2root = [0]
        self.numNodes = 1

    ##################################################
    # methods with obstacles
    ##################################################

    def setObstacles(self, obsList = []):
        self.obsList = obsList

    def edgeCollision(self, coor1, coor2, multiplier = 1):
        interpPoints = self.interpPoints * multiplier
        interpCoor = [coor2[0], coor2[1]]
        dx = (coor1[0] - coor2[0]) / interpPoints
        dy = (coor1[1] - coor2[1]) / interpPoints
        for i in range(0, interpPoints + 1):
            if self.pointCollision(interpCoor):
                return True
            interpCoor[0] += dx
            interpCoor[1] += dy
        return False

    def pointCollision(self, coor):
        obsList = self.obsList.copy()
        while len(obsList) > 0:
            obstacle = obsList.pop(0)
            if obstacle.collidepoint(coor[0], coor[1]):
                return True
        return False

    ##################################################
    # methods for manipulating nodes and edges
    ##################################################

    def connectValidNode(self, childCoor, parentID):
        parentCoor = self.getCoor(parentID)
        if self.edgeCollision(childCoor, parentCoor):
            return False
        else:
            childID = self.numNodes
            self.addNode(childCoor, childID)
            self.addEdge(childID, parentID)
            return True

    def removeNode(self, nodeID):
        for i in range(self.numNodes):
            if self.parentIDs[i] > nodeID: self.parentIDs[i] -= 1
            for j in range(len(self.childIDs[i])):
                if self.childIDs[i][j] > nodeID: self.childIDs[i][j] -= 1
        for i in range(len(self.goalID)):
            if self.goalID[i] > nodeID: self.goalID[i] -= 1

        self.removeEdge(nodeID)
        self.x.pop(nodeID)
        self.y.pop(nodeID)
        self.parentIDs.pop(nodeID)
        self.childIDs.pop(nodeID)
        self.dist2root.pop(nodeID)
        self.numNodes -= 1

    def interpConnect(self, childCoor, parentID, interpPoints):
        coor2 = self.getCoor(parentID)
        interpCoor = [coor2[0], coor2[1]]
        dx = (childCoor[0] - coor2[0]) / interpPoints
        dy = (childCoor[1] - coor2[1]) / interpPoints
        for i in range(0, interpPoints - 1):
            interpCoor[0] += dx
            interpCoor[1] += dy
            childID = self.numNodes
            #self.addNode(interpCoor, childID)
            #self.addEdge(childID, parentID)
            if self.connectValidNode(interpCoor, parentID):
                parentID = childID
        #childID = self.numNodes
        #self.addNode(childCoor, childID)
        #self.addEdge(childID, parentID)
        self.connectValidNode(childCoor, parentID)
        return False

    def furthestConnect(self, childCoor, parentID):
        coor2 = self.getCoor(parentID)
        interpCoor = [coor2[0], coor2[1]]
        dist = self.distance(interpCoor, childCoor)
        n = math.floor(dist/self.maxStep) + 1
        dx = (childCoor[0] - coor2[0]) / n
        dy = (childCoor[1] - coor2[1]) / n
        interpCoor[0] += dx
        interpCoor[1] += dy
        while n > 1 and self.connectValidNode(interpCoor, parentID):
            interpCoor[0] += dx
            interpCoor[1] += dy
            parentID = self.numNodes - 1
            n -= 1
        if n == 1:
            self.connectValidNode(childCoor, parentID)

    ##################################################
    # node finding methods
    ##################################################

    def sampleUnitCircle(self):
        coor = [rnd.uniform(-1,1), rnd.uniform(-1,1)]
        '''
        th = rnd.uniform(0, 1) * 2 * math.pi
        r = math.sqrt(rnd.uniform(0, 1))
        coor = [r * math.cos(th), r * math.sin(th)]
        #'''
        return coor

    def calcEllipseTransformation(self):
        cb = self.goalDist2root[-1]
        cm = self.distance(self.root, self.goal)
        val = cb*cb - cm*cm
        m = 0
        if val > 0:
            m = math.sqrt(cb*cb - cm*cm)
        th = math.atan2(self.root[1] - self.goal[1], self.root[0] - self.goal[0])
        mid = self.interpCoor(self.root, self.goal)
        cth = math.cos(th)
        sth = math.sin(th)
        self.ellipseMatrix = [[cb*cth, -m*sth, mid[0]], [cb*sth, m*cth, mid[1]]]
        return self.ellipseMatrix

    def sampleEllipse(self):
        if self.ellipseMatrix != None:
            M = self.ellipseMatrix
            newCoor = None
            resample = True
            while resample:
                coor = self.sampleUnitCircle()
                newCoor = [ M[0][0] * coor[0] + M[0][1] * coor[1] + M[0][2] , M[1][0] * coor[0] + M[1][1] * coor[1] + M[1][2] ]
                if newCoor[0] > self.minCoor[0] or newCoor[1] > self.minCoor[1] or \
                        newCoor[0] < self.minCoor[0] + self.mapDim[0] or newCoor[1] < self.minCoor[1] + self.mapDim[1]:
                    resample = False
        else:
            newCoor = self.sampleCoor()

        if self.dist2(self.goal, newCoor) < self.maxStep ** 2:
            newCoor = self.goal
        return newCoor

    def sampleCoor(self):
        coor = [rnd.uniform(0, self.mapDim[0]) + self.minCoor[0],
                rnd.uniform(0, self.mapDim[1]) + self.minCoor[1]]
        return coor

    def smartSampleCoor(self):
        dmin2 = 0
        thresh = self.densityDist**2
        while dmin2 < thresh:
            coor = self.sampleEllipse()
            dmin2 = self.dist2(self.root, coor)
            nearID = 0
            for nodeID in range(0, self.numNodes-1):
                nodeCoor = self.getCoor(nodeID)
                dist2 = self.dist2(nodeCoor, coor)

                #'''
                goalIsNotParent = True
                for ID in self.goalID:
                    if nodeID == ID: goalIsNotParent = False
                #'''

                if dist2 < dmin2 and goalIsNotParent:
                    dmin2 = dist2
                    nearID = nodeID
                    if dist2 < thresh:
                        break
            if self.dist2(self.goal, coor) < self.toGoalStep**2 and not self.edgeCollision(self.goal, coor):
                coor = self.goal
                break
        return coor, nearID

    def nearestNodeID(self, coor):
        rootCoor = self.getCoor(0)
        dmin2 = self.dist2(rootCoor, coor)
        nearID = 0
        for nodeID in range(0, self.numNodes-1):
            nodeCoor = self.getCoor(nodeID)
            dist2 = self.dist2(nodeCoor, coor)

            '''
            goalIsNotParent = True
            for ID in self.goalID:
                if nodeID == ID: goalIsNotParent = False
                if self.parentIDs[ID] == nodeID: goalIsNotParent = False
            #'''

            if dist2 < dmin2: # and goalIsNotParent:
                dmin2 = dist2
                nearID = nodeID
        return nearID

    def step(self, randCoor, nearID):
        nearCoor = self.getCoor(nearID)
        d = self.distance(nearCoor, randCoor)
        if d > self.neighbourSize:
            u = self.neighbourSize/d
            near2rand = [randCoor[0] - nearCoor[0], randCoor[1] - nearCoor[1]]
            randCoor = [nearCoor[0] + near2rand[0] * u ,
                        nearCoor[1] + near2rand[1] * u ]

        if self.dist2(randCoor, self.goal) < self.neighbourSize**2:
            return self.goal
        else:
            return randCoor

    def smartStep(self, childCoor):
        minNBdist2root = 1000*self.distance(self.root, self.goal)
        NBDist2 = (self.neighbourSize)**2
        minID = None
        minDist = None

        dist = self.distance(childCoor, self.goal)
        if dist < self.maxStep:
            childCoor = self.goal

        nodeIDs = range(self.numNodes - 1)
        if self.numNodes == 1:
            nodeIDs = [0]

        for nodeID in nodeIDs:
            nodeCoor = self.getCoor(nodeID)
            dist2 = self.dist2(nodeCoor, childCoor)
            if dist2 < NBDist2:
                value = self.dist2root[nodeID]
                dist = math.sqrt(dist2)
                dist2root = value + dist
                # find min NBdis2root
                if dist2root < minNBdist2root:
                    notGoalParent = True
                    for goalID in self.goalID:
                        if nodeID == self.parentIDs[goalID]:
                            notGoalParent = False
                            break
                    if notGoalParent and not self.edgeCollision(self.getCoor(nodeID),childCoor, math.floor(dist/self.maxStep)+1):
                        minNBdist2root = dist2root
                        minID = nodeID
                        minDist = dist

        if minID != None:
            #self.connectValidNode(childCoor, minID)
            self.furthestConnect(childCoor, minID)

    ##################################################
    # RRT star methods
    ##################################################

    def prune(self, value):
        for i in range(self.numNodes-1,1,-1):
            potential = self.dist2root[i] + self.distance(self.getCoor(i), self.goal)
            if potential > value:
                self.removeNode(i)

    def update(self, parentID, change):
        self.dist2root[parentID] += change
        self.counter += 1
        #print(self.counter)
        print(parentID)
        parents = []
        for childID in self.childIDs[parentID]:
            parents.append(self.parentIDs[childID])
        print(parents)
        for childID in self.childIDs[parentID]:
            self.update(childID, change)

    def rewire(self, childCoor):
        NBDist2 = (self.neighbourSize)**2
        NBIDs = []
        NBdist2root = []
        minInd = None
        minID = 0
        minNBdist2root = 1000*self.distance(self.root, self.goal)

        nodeIDs = range(self.numNodes - 1)
        if self.numNodes == 1:
            nodeIDs = [0]

        for nodeID in nodeIDs:
            nodeCoor = self.getCoor(nodeID)
            dist2 = self.dist2(nodeCoor, childCoor)
            if dist2 < NBDist2:
                NBIDs.append(nodeID)
                value = self.dist2root[nodeID]
                dist2root = value + dist2**0.5
                NBdist2root.append(dist2root)
                # find min NBdis2root
                if dist2root < minNBdist2root and not self.edgeCollision(self.getCoor(nodeID),childCoor):
                    minInd = len(NBdist2root) - 1
                    minNBdist2root = dist2root
                    minID = nodeID


        if minInd != None:
            NBdist2root.pop(minInd)
            self.connectValidNode(childCoor, minID)
            #'''
            newID = self.numNodes - 1
            for i in range(len(NBIDs)-1):
                currentVal = self.dist2root[NBIDs[i]]
                change = self.dist2root[newID] + self.distance(childCoor, self.getCoor(NBIDs[i])) - currentVal
                if change < 0:

                    attached2root = True
                    newpos = self.parentIDs[newID]
                    while newpos != 0:
                        if newpos == NBIDs[i]:
                            attached2root = False
                        newpos = self.parentIDs[newpos]

                    if attached2root:
                        self.removeEdge(NBIDs[i])
                        self.addEdge(NBIDs[i], newID)
                        for childID in self.childIDs[NBIDs[i]]:
                            self.update(childID, change)
            #'''

    ##################################################
    # Tree building methods
    ##################################################

    def expand(self):
        oldID = self.numNodes
        newCoor = self.sampleEllipse()
        self.smartStep(newCoor)
        #self.rewire(newCoor)

        childCoor = []
        parentCoor = []
        for i in range(oldID, self.numNodes - 1):
            childCoor.append(self.getCoor(i))
            parentCoor.append(self.getCoor(self.parentIDs[i]))
        return childCoor, parentCoor

    def potentialBias(self, newCoor):
        oldID = self.numNodes

        bestPotential = 100000*self.distance(self.root, self.goal)
        bestID = 0
        bestDist = 0

        n = self.numNodes - 1
        if n == 0: n = 1
        for i in range(n):
            potential = self.dist2root[i]
            dist = self.distance(self.getCoor(i), newCoor)
            potential += dist

            if dist < self.neighbourSize:
                notGoalParent = True
                for goalID in self.goalID:
                    if i == goalID or i == self.parentIDs[goalID]:
                        notGoalParent = False
                        break

                if potential < bestPotential and notGoalParent and not self.edgeCollision(self.getCoor(i), self.goal, math.floor(dist/self.maxStep)+1):
                    bestPotential = potential
                    bestID = i
                    bestDist = dist
        self.furthestConnect(newCoor, bestID)

        # get last coordinate
        childCoor = []
        parentCoor = []
        for i in range(oldID, self.numNodes - 1):
            childCoor.append(self.getCoor(i))
            parentCoor.append(self.getCoor(self.parentIDs[i]))
        return childCoor, parentCoor

    def goalBias(self):
        bestPotential = 1000000*self.distance(self.root, self.goal)
        bestID = 0
        for i in range(self.numNodes):
            potential = self.dist2root[i]
            dist2goal = self.distance(self.getCoor(i), self.goal)
            if dist2goal < self.neighbourSize:
                potential += dist2goal

                goalIsNotParent = True
                for ID in self.goalID:
                    if i == ID: goalIsNotParent = False
                    if self.parentIDs[ID] == i: goalIsNotParent = False
                if potential < bestPotential and goalIsNotParent:
                    bestPotential = potential
                    bestID = i

        self.step(self.goal, bestID)

        # get last coordinate
        childCoor = self.getCoor(-1)
        parentID = self.parentIDs[-1]
        parentCoor = self.getCoor(parentID)
        return childCoor, parentCoor

    def rrt_expand(self):
        newCoor, parentID = self.smartSampleCoor()
        childCoor = self.step(newCoor, parentID)
        self.connectValidNode(childCoor, parentID)

        childCoor = self.getCoor(-1)
        parentID = self.parentIDs[-1]
        parentCoor = self.getCoor(parentID)
        return childCoor, parentCoor

    def rrt_bias(self, newCoor):
        parentID = self.nearestNodeID(newCoor)
        childCoor = self.step(newCoor, parentID)
        self.connectValidNode(childCoor, parentID)

        # get last coordinate
        childCoor = self.getCoor(-1)
        parentID = self.parentIDs[-1]
        parentCoor = self.getCoor(parentID)
        return childCoor, parentCoor

    ##################################################
    # Path methods
    ##################################################

    def goalPath(self, pathNum = 0):
        return self.path2root(self.goalID[pathNum])

    def goalDist(self, pathNum):
        return self.getDist2root(self.goalID[pathNum])

    def calcPathDist(self, path):
        oldCoor = None
        dist = 0
        for coor in path:
            if oldCoor != None:
                dist += self.distance(coor, oldCoor)
            oldCoor = coor
        return dist

    ##################################################
    # vector methods
    ##################################################

    def dot(self, vect1, vect2):
        return vect1[0] * vect2[0] + vect1[1] * vect2[1]

    def project(self, vect1, vect2):
        K = self.dot(vect1, vect2)/self.dot(vect2, vect2)
        return [K * vect2[0], K * vect2[1]]

    def interpCoor(self, coor1, coor2, x = 0.5):
        vect = [coor2[0] - coor1[0], coor2[1] - coor1[1]]
        interpCoor = [coor1[0] + x * vect[0], coor1[1] + x * vect[1]]
        return interpCoor

    ##################################################
    # path solver methods
    ##################################################

    def setSolveParam(self, maxStep = 10, neighbourSize = 3, toGoalStep = 0.5, interpPoints = 10, sampleLength = 10, shortenIter = 200):
        self.interpPoints = interpPoints
        self.maxStep = maxStep
        self.neighbourSize = neighbourSize
        self.toGoalStep = toGoalStep
        self.densityDist = maxStep
        self.shortenIter = shortenIter
        self.sampleLength = sampleLength

    def shortenPath(self, path, K_t = 1, K_n = 1):
        newPath = [] + path
        noChangeID = []
        for k in range(self.shortenIter):
            for i in range(1, len(newPath)-1):
                secant = [path[i+1][0] - newPath[i-1][0], path[i+1][1] - newPath[i-1][1]]
                midpoint = [newPath[i-1][0] + 0.5 * secant[0], newPath[i-1][1] + 0.5 * secant[1]]
                i2midpoint = [midpoint[0] - path[i][0], midpoint[1] - path[i][1]]

                # project i2midpoint onto secant and normal vector
                dt = self.project(i2midpoint, secant)
                dt = [K_t * dt[0], K_t * dt[1]]
                dn = [i2midpoint[0] - dt[0], i2midpoint[1] - dt[1]]
                dn = [K_n * dn[0], K_n * dn[1]]

                # move tangentially first, then "normally"
                coor = [path[i][0], path[i][1]]
                #'''
                coor = [coor[0] + dt[0], coor[1] + dt[1]]
                if not self.pointCollision(coor):
                    newPath[i] = coor
                # '''
                coor = [coor[0] + dn[0], coor[1] + dn[1]]
                if not self.pointCollision(coor):
                    newPath[i] = coor
                elif k == self.shortenIter-1:
                    noChangeID = [i] + noChangeID

            path = [] + newPath
        return newPath, noChangeID

    def resamplePath(self, path, dist2root):
        numNodes = dist2root/self.sampleLength
        reduction = len(path)/numNodes
        newPath = path + []
        if reduction >= 2:
            reps = math.floor(math.log2(reduction))
            for k in range(reps-1):
                for i in range(1, math.floor(0.5*len(newPath))):
                    newPath.pop(i)
        elif reduction < 1:
            reps = 1 - math.floor(math.log2(reduction))
            for k in range(reps-1):
                for i in range(len(newPath)-1, 0, -1):
                    newCoor = self.interpCoor(newPath[i], newPath[i-1])
                    newPath.insert(i, newCoor)

        return newPath

    def removeIDs(self, path, IDs):
        newPath = []
        newPath = [path[-1]] + newPath
        for i in IDs:
            newPath = [path[i]] + newPath
        newPath = [path[0]] + newPath
        return newPath

    def refinePath(self):
        solutionPath = self.path2root(-1)
        solutionPath = self.resamplePath(solutionPath, self.dist2root[-1])
        solutionPath, noChangeIDs = self.shortenPath(solutionPath)
        #solutionPath = self.removeIDs(solutionPath, noChangeIDs)
        #dist = self.dist2root[-1]
        dist = self.calcPathDist(solutionPath)
        self.goalID.append(self.numNodes - 1)
        self.goalDist2root.append(dist)
        self.calcEllipseTransformation()
        self.prune(self.goalDist(-1))
        return solutionPath, dist

    def solve(self, start, goal, rep = 3000, map = None, drawAll = False):
        self.reset(start)
        self.goal = goal
        if not map == None:
            map.drawMap()

        distStart2goal = self.distance(self.root, self.goal)
        distStart2goal += 1.01
        minPathDist = 1000000*distStart2goal
        iter = 0
        while len(self.goalID) == 0 or iter < rep:
            iter += 1
            if iter % 10 == 0:
                childCoor, parentCoor = self.potentialBias(self.goal)
            else:
                childCoor, parentCoor = self.expand()
            if self.x[-1] == self.goal[0] and self.y[-1] == self.goal[1]:
                dist = self.dist2root[-1]
                if dist < minPathDist - self.maxStep/1000:
                    solutionPath, minPathDist = self.refinePath()
                    if not map == None:
                        if drawAll:
                            map.drawMap()
                            map.drawTree(self.x, self.y, self.parentIDs)
                        map.drawPath(solutionPath)
                        print("\nID: {}".format(len(self.goalDist2root)-1))
                        print("Num of nodes: {}".format(self.numNodes))
                        print("Path length: {}".format(self.goalDist2root[len(self.goalID) - 1]))
                    if dist < distStart2goal: break
            if drawAll and not map == None:
                for i in range(len(childCoor)-1):
                    map.drawCircle(map.grey, childCoor[i], 2, 0)
                    map.drawLine(map.blue, childCoor[i], parentCoor[i], 1)

        if len(self.goalID) > 0:
            #solutionPath, noChangeIDs = self.shortenPath(solutionPath)
            #solutionPath = self.removeIDs(solutionPath, noChangeIDs)
            if not map == None:
                map.drawPath(solutionPath, map.red)
                print("\nMin Special_RRT distance:")
                print(minPathDist)

        return solutionPath, minPathDist

    ##################################################
    # backtracking methods
    ##################################################

    def backtrackCollision(self, childCoor, parentID):
        backCoor = [self.backtrackSteps*(parentID[0] - childCoor[0]), self.backtrackSteps*(parentID[1] - childCoor[1])]
        backCoor = [backCoor[0] + parentID[0], backCoor[1] + parentID[1]]
        return self.edgeCollision(backCoor, parentID, self.backtrackSteps*self.interpPoints)

    def backtrackValidNode(self, childCoor, parentID):
        parentCoor = self.getCoor(parentID)
        if self.edgeCollision(childCoor, parentCoor) or self.backtrackCollision(childCoor, parentCoor):
            return False
        else:
            childID = self.numNodes
            self.addNode(childCoor, childID)
            self.addEdge(childID, parentID)
            return True

    def backtrackSmartStep(self, childCoor):
        minNBdist2root = 1000 * self.distance(self.root, self.goal)
        NBDist2 = (self.neighbourSize) ** 2
        minID = None
        minDist = None

        dist = self.distance(childCoor, self.goal)
        if dist < self.maxStep:
            childCoor = self.goal

        nodeIDs = range(self.numNodes - 1)
        if self.numNodes == 1:
            nodeIDs = [0]

        for nodeID in nodeIDs:
            nodeCoor = self.getCoor(nodeID)
            dist2 = self.dist2(nodeCoor, childCoor)
            if dist2 < NBDist2:
                value = self.dist2root[nodeID]
                dist = math.sqrt(dist2)
                dist2root = value + dist
                # find min NBdis2root
                if dist2root < minNBdist2root:
                    notGoalParent = True
                    for goalID in self.goalID:
                        if nodeID == self.parentIDs[goalID]:
                            notGoalParent = False
                            break
                    if notGoalParent and not self.edgeCollision(self.getCoor(nodeID), childCoor,
                            math.floor(dist / self.maxStep) + 1) and not self.backtrackCollision(childCoor, self.getCoor(nodeID)):
                        minNBdist2root = dist2root
                        minID = nodeID
                        minDist = dist

        if minID != None:
            # self.connectValidNode(childCoor, minID)
            self.furthestConnect(childCoor, minID)

    def backtrackExpand(self):
        oldID = self.numNodes
        newCoor = self.sampleEllipse()
        self.backtrackSmartStep(newCoor)
        #self.rewire(newCoor)

        childCoor = []
        parentCoor = []
        for i in range(oldID, self.numNodes - 1):
            childCoor.append(self.getCoor(i))
            parentCoor.append(self.getCoor(self.parentIDs[i]))
        return childCoor, parentCoor

    def backPotentialBias(self, newCoor):
        oldID = self.numNodes

        bestPotential = 100000*self.distance(self.root, self.goal)
        bestID = 0
        bestDist = 0

        n = self.numNodes - 1
        if n == 0: n = 1
        for i in range(n):
            potential = self.dist2root[i]
            dist = self.distance(self.getCoor(i), newCoor)
            potential += dist

            if dist < self.neighbourSize:
                notGoalParent = True
                for goalID in self.goalID:
                    if i == goalID or i == self.parentIDs[goalID]:
                        notGoalParent = False
                        break

                if potential < bestPotential and notGoalParent and not self.edgeCollision(self.getCoor(i),
                        self.goal, math.floor(dist/self.maxStep)+1) and self.backtrackCollision(self.goal, self.getCoor(i)):
                    bestPotential = potential
                    bestID = i
                    bestDist = dist
        self.furthestConnect(newCoor, bestID)

        # get last coordinate
        childCoor = []
        parentCoor = []
        for i in range(oldID, self.numNodes - 1):
            childCoor.append(self.getCoor(i))
            parentCoor.append(self.getCoor(self.parentIDs[i]))
        return childCoor, parentCoor

    def backtrackShortenPath(self, path, K_t = 1, K_n = 1):
        newPath = [] + path
        change = [True] * len(path)
        for k in range(self.shortenIter):
            for i in range(1, len(newPath)-1):
                if change[i]:
                    secant = [path[i+1][0] - newPath[i-1][0], path[i+1][1] - newPath[i-1][1]]
                    midpoint = [newPath[i-1][0] + 0.5 * secant[0], newPath[i-1][1] + 0.5 * secant[1]]
                    i2midpoint = [midpoint[0] - path[i][0], midpoint[1] - path[i][1]]

                    # project i2midpoint onto secant and normal vector
                    dt = self.project(i2midpoint, secant)
                    dt = [K_t * dt[0], K_t * dt[1]]
                    dn = [i2midpoint[0] - dt[0], i2midpoint[1] - dt[1]]
                    dn = [K_n * dn[0], K_n * dn[1]]

                    # move tangentially first, then "normally"
                    coor = [path[i][0], path[i][1]]
                    #'''
                    coor = [coor[0] + dt[0], coor[1] + dt[1]]
                    if not self.pointCollision(coor) and not self.backtrackCollision(newPath[i+1],coor) and not self.backtrackCollision(coor, newPath[i-1]):
                        newPath[i] = coor
                    # '''
                    coor = [coor[0] + dn[0], coor[1] + dn[1]]
                    if not self.pointCollision(coor) and not self.backtrackCollision(newPath[i+1],coor) and not self.backtrackCollision(coor, newPath[i-1]):
                        newPath[i] = coor
                    #else: change[i] = False

            path = [] + newPath
        return newPath

    def backtrackRefine(self):
        solutionPath = self.path2root(-1)
        #solutionPath = self.resamplePath(solutionPath, self.dist2root[-1])
        #solutionPath = self.backtrackShortenPath(solutionPath)
        dist = self.calcPathDist(solutionPath)
        self.goalID.append(self.numNodes - 1)
        self.goalDist2root.append(dist)
        self.calcEllipseTransformation()
        return solutionPath, dist

    def solveWithBacktrack(self, start, goal, backtrackSteps = 5, rep = 2000, map = None, drawAll = False):
        self.reset(start)
        self.goal = goal
        self.backtrackSteps = backtrackSteps
        if not map == None:
            map.drawMap()

        distStart2goal = self.distance(self.root, self.goal)
        distStart2goal += 1.01
        minPathDist = 1000000 * distStart2goal
        iter = 0
        while len(self.goalID) == 0 or iter < rep:
            iter += 1
            if iter % 5 == 0:
                childCoor, parentCoor = self.backPotentialBias(self.goal)
            else:
                childCoor, parentCoor = self.backtrackExpand()
            if self.x[-1] == self.goal[0] and self.y[-1] == self.goal[1]:
                dist = self.dist2root[-1]
                if dist < minPathDist - self.maxStep / 1000:
                    solutionPath, minPathDist = self.backtrackRefine()
                    if not map == None:
                        if drawAll:
                            map.drawMap()
                            map.drawTree(self.x, self.y, self.parentIDs)
                        map.drawPath(solutionPath)
                        print("\nID: {}".format(len(self.goalDist2root) - 1))
                        print("Num of nodes: {}".format(self.numNodes))
                        print("Path length: {}".format(self.goalDist2root[len(self.goalID) - 1]))
                    if dist < distStart2goal: break
            if drawAll and not map == None:
                for i in range(len(childCoor) - 1):
                    map.drawCircle(map.grey, childCoor[i], 2, 0)
                    map.drawLine(map.blue, childCoor[i], parentCoor[i], 1)

        if len(self.goalID) > 0:
            # solutionPath, noChangeIDs = self.shortenPath(solutionPath)
            # solutionPath = self.removeIDs(solutionPath, noChangeIDs)
            if not map == None:
                map.drawPath(solutionPath, map.red)
                print("\nMin Special_RRT distance:")
                print(minPathDist)

        return solutionPath, minPathDist





##################################################
# testing
##################################################

def main():
    import pygame
    from Obstacles import Obstacles
    from Map import Map
    import time as TIME
    import numpy as np

    mapDim = [600, 600]
    start = [50,50]
    goal = [mapDim[0]-50, mapDim[1]-50]
    obsDim = 40
    obsNum = 30

    obsGenerator = Obstacles()
    rrt = RRT_star(mapDim)
    map = Map(mapDim)
    map.setStartGoal(start, goal)



    iter = 20
    rounds = 5
    time = np.zeros([rounds, iter])
    dist = np.zeros([rounds, iter])
    aveTime = np.zeros(rounds)
    aveDist = np.zeros(rounds)
    for i in range(iter):
        print("\nITERATION {}:".format(i))
        obsList = obsGenerator.makeNoneCollideObs(mapDim, start, goal, obsDim, obsNum)
        rrt.setObstacles(obsList)
        map.setObstacles(obsList)

        for j in range(rounds):
            begin = TIME.time()
            rep = 100 + 100 * j
            rrt.setSolveParam(20, 300, 50, 5, 20, 200)
            solutionPath, dist[j][i] = rrt.solve(start, goal, rep, map, True)
            end = TIME.time()
            time[j][i] = end - begin

            print("time {}: {}".format(j, time[j][i]))
            print("distance {}: {}".format(j, dist[j][i]))
            aveDist[j] += dist[j][i]
            aveTime[j] += time[j][i]

    print("\n\n\nOVERALL COMPARISONS: ")
    for j in range(rounds):
        print("..................")
        print("Ave distance {}: {}".format(j, aveDist[j] / iter))
        print("Ave time {}: {}".format(j, aveTime[j] / iter))


    print("\n\n\ntime:")
    for j in range(rounds):
        print(time[j])
    print("dist")
    for j in range(rounds):
        print(dist[j])

if __name__ == '__main__':
    main()