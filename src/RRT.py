from treeStructure.basicCoordinateTree import BasicCoordinateTree
import random as rnd
import math
import pygame

# RRT is a subclass of CoordinateTree
class RRT(BasicCoordinateTree):
    def __init__(self, mapDim, minCoor = [0,0], obsList = []):
        # inherit methods and values from
        super().__init__([0,0])
        self.goal = [0,0]
        self.goalID = []
        self.goalDist2root = []
        self.path = []
        self.straitDist2root = 0

        self.obsList = obsList

        self.mapDim = mapDim
        self.minCoor = minCoor

        self.interpPoints = 1
        self.maxStep = 10
        self.shortenIter = 100
        self.backtrackSteps = 3

        self.ellipseMatrix = None
        self.sampleLength = 10
        self.pushObjectSize = 10

    def reset(self, root):
        self.root = root
        self.x = [root[0]]
        self.y = [root[1]]
        self.parentIDs = [0]
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

    def edgeCollision(self, coor1, coor2, interpPoints = None):
        if interpPoints == None:
            interpPoints = self.interpPoints
        interpCoor = [coor2[0], coor2[1]]
        dx = (coor1[0] - coor2[0]) / interpPoints
        dy = (coor1[1] - coor2[1]) / interpPoints
        for i in range(interpPoints-1):
            if self.pointCollision(interpCoor):
                return True
            interpCoor[0] += dx
            interpCoor[1] += dy
        if self.pointCollision(coor1):
            return True
        return False

    def pointCollision(self, coor):
        obsList = self.obsList.copy()
        while len(obsList) > 0:
            obstacle = obsList.pop(0)
            if obstacle.collidepoint(coor[0], coor[1]):
                return True
        return False

    def prune(self, value):
        for i in range(self.numNodes-1,1,-1):
            if self.dist2root[i] > value:
                self.removeNode(i)

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

    def calcEllipseTransformation(self, dist = None):
        cb = dist
        if dist == None:
            cb = self.goalDist2root[-1]
        cm = self.distance(self.root, self.goal)
        val = cb * cb - cm * cm
        m = 0
        if val > 0:
            m = math.sqrt(cb * cb - cm * cm)
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
        thresh = 0.25*self.maxStep**2
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

                if dist2 < dmin2:# and goalIsNotParent:
                    dmin2 = dist2
                    nearID = nodeID
                    if dist2 < thresh:
                        break
            if self.dist2(self.goal, coor) < self.maxStep**2:
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
            if dist2 < dmin2:
                dmin2 = dist2
                nearID = nodeID
        return nearID

    def step(self, randCoor, nearID):
        nearCoor = self.getCoor(nearID)
        d = self.distance(nearCoor, randCoor)
        if d > self.maxStep:
            u = self.maxStep/d
            near2rand = [randCoor[0] - nearCoor[0], randCoor[1] - nearCoor[1]]
            randCoor = [nearCoor[0] + near2rand[0] * u ,
                        nearCoor[1] + near2rand[1] * u ]

        if self.dist2(randCoor, self.goal) < self.maxStep**2:
            return self.goal
        else:
            return randCoor

    def smartStep(self, childCoor, neighbourhoodSize = 5):
        NBDist2 = (neighbourhoodSize * self.maxStep)**2
        minID = None
        minNBdist2root = 1000*self.distance(self.root, self.goal)

        nodeIDs = range(self.numNodes - 1)
        if self.numNodes == 1:
            nodeIDs = [0]

        for nodeID in nodeIDs:
            nodeCoor = self.getCoor(nodeID)
            dist2 = self.dist2(nodeCoor, childCoor)
            if dist2 < NBDist2:
                value = self.dist2root[nodeID]
                dist2root = value + math.sqrt(dist2)
                # find min NBdis2root
                if dist2root < minNBdist2root and not self.edgeCollision(self.getCoor(nodeID),childCoor):
                    minNBdist2root = dist2root
                    minID = nodeID


        #if minID != None:
            #self.connectValidNode(childCoor, minID)
        return childCoor


    ##################################################
    # Tree building methods
    ##################################################

    def expand(self):
        newCoor, parentID = self.smartSampleCoor()
        childCoor = self.step(newCoor, parentID)
        self.connectValidNode(childCoor, parentID)
        #self.smartStep(newCoor)

        childCoor = self.getCoor(-1)
        parentID = self.parentIDs[-1]
        parentCoor = self.getCoor(parentID)
        return childCoor, parentCoor

    def bias(self, newCoor):
        parentID = self.nearestNodeID(newCoor)
        childCoor = self.step(newCoor, parentID)
        self.connectValidNode(childCoor, parentID)
        #self.smartStep(newCoor)

        # get last coordinate
        childCoor = self.getCoor(-1)
        parentID = self.parentIDs[-1]
        parentCoor = self.getCoor(parentID)
        return childCoor, parentCoor

    ##################################################
    # retrieve path methods
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
        K = self.dot(vect1, vect2) / self.dot(vect2, vect2)
        return [K * vect2[0], K * vect2[1]]

    def interpCoor(self, coor1, coor2, x=0.5):
        vect = [coor2[0] - coor1[0], coor2[1] - coor1[1]]
        interpCoor = [coor1[0] + x * vect[0], coor1[1] + x * vect[1]]
        return interpCoor

    ##################################################
    # path solver methods
    ##################################################

    def verify(self, path):
        oldCoor = None
        olderCoor = None
        for coor in path:
            if olderCoor != None:
                dist = self.distance(coor, oldCoor)
                n = math.floor(dist/self.maxStep) + 1
                if self.edgeCollision(coor, oldCoor, n):
                    return False
            oldCoor = coor
            olderCoor = oldCoor
        return True

    def setSolveParam(self, maxStep = 10, interpPoints = 1, sampleLength = 10, shortenIter = 100):
        self.interpPoints = interpPoints
        self.maxStep = maxStep
        self.shortenIter = shortenIter
        self.sampleLength = sampleLength

    def shortenPath(self, path, K_t = 1, K_n = 1):
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
                    if not self.pointCollision(coor):
                        newPath[i] = coor
                    # '''
                    coor = [coor[0] + dn[0], coor[1] + dn[1]]
                    if not self.pointCollision(coor):
                        newPath[i] = coor
                    else: change[i] = False

            path = [] + newPath
        return newPath

    def snapPath2obstacle(self, path):
        newPath = []
        start = path[0]
        goal = path[-1]
        newPath.append(start)
        path = path[1:len(path)-1]
        oldCoor = start
        for coor in path:
            minDist2 = 4*self.maxStep ** 2
            minCoor = [0,0]
            for obs in self.obsList:
                dist2 = self.dist2(obs.bottomleft, coor)
                if dist2 < minDist2:
                    minDist2 = dist2
                    minCoor = [obs.bottomleft[0] + 0.05 * (obs.bottomleft[0] - obs.center[0]),
                               obs.bottomleft[1] + 0.05 * (obs.bottomleft[1] - obs.center[1])]
                dist2 = self.dist2(obs.topleft, coor)
                if dist2 < minDist2:
                    minDist2 = dist2
                    minCoor = [obs.topleft[0] + 0.05 * (obs.topleft[0] - obs.center[0]),
                               obs.topleft[1] + 0.05 * (obs.topleft[1] - obs.center[1])]
                dist2 = self.dist2(obs.bottomright, coor)
                if dist2 < minDist2:
                    minDist2 = dist2
                    minCoor = [obs.bottomright[0] + 0.05 * (obs.bottomright[0] - obs.center[0]),
                               obs.bottomright[1] + 0.05 * (obs.bottomright[1] - obs.center[1])]
                dist2 = self.dist2(obs.topright, coor)
                if dist2 < minDist2:
                    minDist2 = dist2
                    minCoor = [obs.topright[0] + 0.05 * (obs.topright[0] - obs.center[0]),
                               obs.topright[1] + 0.05 * (obs.topright[1] - obs.center[1])]
            if oldCoor != minCoor:
                newPath.append(minCoor)
            oldCoor = minCoor
        newPath.append(goal)
        return newPath

    def shortenReducePath(self, path, K_t = 1, K_n = 1):
        newPath = [] + path
        noChangeID = [0,len(path)-1]
        noChange = [False]*len(path)
        noChange[0] = True
        noChange[-1] = True
        iter = 0
        n = math.floor(self.straitDist2root/self.maxStep) + 1
        if not self.edgeCollision(self.root, self.goal, n):
            iter = self.shortenIter
            newPath = [path[0], path[-1]]
        while iter < self.shortenIter and len(noChangeID) < len(newPath):
            iter += 1
            i = 0
            while i < len(newPath)-1:
                i += 1
                if noChange[i]:
                    pass
                else:
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
                    else:
                        noChange[i] = True

                        lastNoChangeID = 0
                        futureNoChangeID = 0
                        noChangeIDind = 0
                        ran = range(len(noChangeID))
                        if len(noChangeID) == 2:
                            ran = [0,1]
                        for j in ran:
                            if i < noChangeID[j]:
                                futureNoChangeID = noChangeID[j]
                                lastNoChangeID = noChangeID[j-1]
                                #noChangeID = noChangeID[0:j] + [i] + noChangeID[j:len(noChangeID)]
                                noChangeID.insert(j,i)
                                noChangeIDind = j
                                break

                        dist = self.distance(newPath[lastNoChangeID], newPath[i])
                        n = math.floor(dist/self.maxStep)
                        if n < 1: n = 1
                        if not self.edgeCollision(newPath[lastNoChangeID], newPath[i], n):
                            j = lastNoChangeID
                            while j < i - 1:
                                j += 1
                                newPath.pop(lastNoChangeID + 1)
                                noChange.pop(lastNoChangeID + 1)
                            j = noChangeIDind - 1
                            while j < len(noChangeID) - 1:
                                j += 1
                                noChangeID[j] = noChangeID[j] - (i - lastNoChangeID - 1)
                            futureNoChangeID -= i - lastNoChangeID - 1
                            i = lastNoChangeID + 1

                        dist = self.distance(newPath[futureNoChangeID], newPath[i])
                        n = math.floor(dist/self.maxStep)
                        if n < 1: n = 1
                        if not self.edgeCollision(newPath[futureNoChangeID], newPath[i], n):
                            j = noChangeIDind
                            while j < len(noChangeID) - 1:
                                j += 1
                                noChangeID[j] = noChangeID[j] - (futureNoChangeID - i - 1)
                            j = i
                            while j < futureNoChangeID - 1:
                                j += 1
                                newPath.pop(i + 1)
                                noChange.pop(i + 1)

                path = [] + newPath
        return newPath

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

    def refinePath(self):
        solutionPath = self.path2root(-1)
        #solutionPath = self.resamplePath(solutionPath, self.dist2root[-1])
        solutionPath = self.shortenReducePath(solutionPath)
        solutionPath = self.snapPath2obstacle(solutionPath)
        #dist = self.dist2root[-1]
        dist = self.calcPathDist(solutionPath)
        self.goalID.append(self.numNodes - 1)
        self.goalDist2root.append(dist)
        #self.calcEllipseTransformation()
        #self.prune(self.goalDist(-1))
        return solutionPath, dist

    def removeIDs(self, path, IDs):
        newPath = path + []
        for i in IDs:
            newPath.pop(i)
        return newPath

    def solve(self, start, goal, rep = 2000, map = None, drawAll = False):
        self.reset(start)
        self.goal = goal
        self.straitDist2root = self.distance(self.root, self.goal)
        if not map == None:
            map.drawMap()

        distStart2goal = 1.01 * self.straitDist2root
        minPathDist = 1000000*distStart2goal
        iter = 0
        solutionPath = None
        while solutionPath == None or iter < rep:
            iter += 1
            if iter % 10 == 1:
                childCoor, parentCoor = self.bias(self.goal)
            else:
                childCoor, parentCoor = self.expand()
            if childCoor[0] == self.goal[0] and childCoor[1] == self.goal[1]:
                sol, dist = self.refinePath()
                if dist < minPathDist - self.maxStep/1000 and self.verify(sol):
                    solutionPath, minPathDist = self.refinePath()
                    self.calcEllipseTransformation(minPathDist)
                    if not map == None:
                        if drawAll:
                            map.drawMap()
                            map.drawTree(self.x, self.y, self.parentIDs)
                        map.drawPath(solutionPath)
                        print("\nID: {}".format(len(self.goalDist2root)-1))
                        print("Num of nodes: {}".format(self.numNodes))
                        print("Path length: {}".format(self.goalDist2root[len(self.goalID) - 1]))
                if dist < distStart2goal: break
                self.removeAllNodes(start)
            if drawAll and not map == None:
                map.drawCircle(map.grey, childCoor, 2, 0)
                map.drawLine(map.blue, childCoor, parentCoor, 1)

        if len(self.goalID) > 0:
            if not map == None:
                map.drawPath(solutionPath, map.red)
                print("\nMin Special_RRT distance:")
                print(minPathDist)

        return solutionPath, minPathDist

    ##################################################
    # backtracking methods
    ##################################################

    def setPushObsSize(self, size):
        self.pushObjectSize = size

    def distInterp(self, coor1, coor2, dist):
        vect = [coor2[0] - coor1[0], coor2[1] - coor1[1]]
        vectNorm = self.distance(coor1, coor2)
        x = dist/vectNorm
        interpCoor = [coor1[0] + x * vect[0], coor1[1] + x * vect[1]]
        return interpCoor

    def addBacktrackingPoints(self, path):
        if path == None:
            return None
        length = len(path)
        forwardTracking = self.distInterp(path[0], path[1], - self.pushObjectSize)
        newPath = [forwardTracking]
        forward = []
        for i in range(1, length-1):
            forward.append(True)
            objectBuffer = self.distInterp(path[i], path[i - 1], 0.75 * self.pushObjectSize)
            newPath.append(objectBuffer)

            forward.append(False)
            backwardTracking = self.distInterp(path[i], path[i-1], self.backtrackSteps*self.maxStep)
            forwardTracking = self.distInterp(path[i], path[i+1], -self.backtrackSteps*self.maxStep)
            pushObs = pygame.Rect([path[i][0] - 0.35*self.backtrackSteps*self.maxStep, path[i][1] - 0.35*self.backtrackSteps*self.maxStep],
                                  (0.7*self.backtrackSteps*self.maxStep, 0.7*self.backtrackSteps*self.maxStep))
            self.obsList.append(pushObs)
            solution, dist = self.solve(backwardTracking, forwardTracking, 1000)
            self.obsList.pop(-1)
            for coor in solution:
                newPath.append(coor)
                forward.append(True)
            forward.pop(-1)

        forward.append(True)
        backwardTracking = self.distInterp(path[-1], path[-2],  self.pushObjectSize)
        newPath.append(backwardTracking)
        return newPath, forward

    def backtrackVerify(self, path):
        oldCoor = None
        olderCoor = None
        for coor in path:
            if olderCoor != None:
                dist = self.distance(coor, oldCoor)
                n = math.floor(dist/self.maxStep) + 1
                if self.backtrackCollision(coor, oldCoor, dist) or self.edgeCollision(coor, oldCoor, n):
                    return False
            oldCoor = coor
            olderCoor = oldCoor
        return True

    def backtrackCollision(self, childCoor, parentCoor, distance = None):
        multiplier = self.backtrackSteps
        if distance != None:
            multiplier = self.backtrackSteps*self.maxStep/distance
        backCoor = [multiplier*(parentCoor[0] - childCoor[0]), multiplier*(parentCoor[1] - childCoor[1])]
        backCoor = [backCoor[0] + parentCoor[0], backCoor[1] + parentCoor[1]]
        return self.edgeCollision(backCoor, parentCoor, math.floor(multiplier*self.interpPoints)+1)

    def backtrackValidNode(self, childCoor, parentID):
        parentCoor = self.getCoor(parentID)
        if self.edgeCollision(childCoor, parentCoor) or self.backtrackCollision(childCoor, parentCoor):
            return False
        else:
            childID = self.numNodes
            self.addNode(childCoor, childID)
            self.addEdge(childID, parentID)
            return True

    def backtrackExpand(self):
        newCoor, parentID = self.smartSampleCoor()
        childCoor = self.smartStep(newCoor, 1)
        self.backtrackValidNode(childCoor, parentID)

        childCoor = self.getCoor(-1)
        parentID = self.parentIDs[-1]
        parentCoor = self.getCoor(parentID)
        return childCoor, parentCoor

    def backtrackBias(self, newCoor):
        parentID = self.nearestNodeID(newCoor)
        childCoor = self.step(newCoor, parentID)
        self.backtrackValidNode(childCoor, parentID)
        # self.smartStep(newCoor)

        # get last coordinate
        childCoor = self.getCoor(-1)
        parentID = self.parentIDs[-1]
        parentCoor = self.getCoor(parentID)
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
                    if not self.pointCollision(coor):
                        if k % 1 == 0:
                            if not self.backtrackCollision(newPath[i+1],coor):
                                newPath[i] = coor
                        elif not self.backtrackCollision(coor, newPath[i-1]):
                            newPath[i] = coor
                    # '''
                    coor = [coor[0] + dn[0], coor[1] + dn[1]]
                    if not self.pointCollision(coor):
                        if k % 1 == 0:
                            if not self.backtrackCollision(newPath[i+1],coor):
                                newPath[i] = coor
                        elif not self.backtrackCollision(coor, newPath[i-1]):
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
        #self.calcEllipseTransformation()
        return solutionPath, dist

    def solvePushObject(self, start, objectCoor, goal, backtrackSteps=5, rep=2000, map=None, drawAll=False):
        pushPath, direction, minPathDist, circleCoor = self.solveWithBacktrack(objectCoor, goal, backtrackSteps, rep)
        if pushPath == None:
            return None, None, None
        pushObs = pygame.Rect([circleCoor[0][0] - 0.5*self.backtrackSteps*self.maxStep, circleCoor[0][1] - 0.5*self.backtrackSteps*self.maxStep],
                              (1*self.backtrackSteps*self.maxStep, 1*self.backtrackSteps*self.maxStep))
        self.obsList.append(pushObs)
        interPoint = self.distInterp(pushPath[0], pushPath[1], - self.backtrackSteps*self.maxStep + self.pushObjectSize)
        navPath, minPathDist = self.solve(start, interPoint, rep)
        self.obsList.pop(-1)

        direction = [True]*(len(navPath)-1) + direction
        solutionPath = navPath
        solutionPath.extend(pushPath[1:len(pushPath)])
        if map != None:
            map.drawMap()
            pygame.draw.rect(map.map, [220, 220, 220], pushObs)
            for coor in circleCoor:
                map.drawCircle(map.blue, coor, math.floor(0.5 * self.pushObjectSize) + 1, 1)
            map.drawPath(solutionPath)
        minPathDist = self.calcPathDist(solutionPath)
        return solutionPath, direction, minPathDist

    def solveWithBacktrack(self, start, goal, backtrackSteps = 5, rep = 2000, map = None, drawAll = False):
        self.reset(start)
        self.goal = goal
        self.backtrackSteps = backtrackSteps
        self.straitDist2root = self.distance(self.root, self.goal)
        if not map == None:
            map.drawMap()

        distStart2goal = 1.01 * self.straitDist2root
        minPathDist = 1000000*distStart2goal
        iter = 0
        solutionPath = None
        direction = None
        circleCoor = None
        while iter < rep:
            iter += 1
            if iter % 100 == 1:
                childCoor, parentCoor = self.bias(self.goal)
            else:
                childCoor, parentCoor = self.expand()
            if self.dist2(childCoor, self.goal) < (0.001 * distStart2goal) ** 2:
                sol, dist = self.refinePath()
                if drawAll:
                    map.drawMap()
                if self.backtrackVerify(sol):
                    if dist < minPathDist - self.maxStep/1000:
                        circleCoor = sol
                        solutionPath = sol
                        minPathDist = dist
                        self.calcEllipseTransformation(minPathDist)
                        if not map == None:
                            map.drawPath(solutionPath)
                            print("\nID: {}".format(len(self.goalDist2root)-1))
                            print("Num of nodes: {}".format(self.numNodes))
                            print("Path length: {}".format(self.goalDist2root[len(self.goalID) - 1]))
                        if dist < distStart2goal: break
                self.removeAllNodes(start)
            if drawAll and not map == None:
                map.drawCircle(map.grey, childCoor, 2, 0)
                map.drawLine(map.blue, childCoor, parentCoor, 1)

        if solutionPath == None:
            return None, None, None, None
        solutionPath, direction = self.addBacktrackingPoints(solutionPath)
        minPathDist = self.calcPathDist(solutionPath)
        if not map == None:
            map.drawMap()
            for coor in circleCoor:
                map.drawCircle(map.blue, coor, math.floor(0.5*self.pushObjectSize)+1, 1)
            map.drawPath(solutionPath, map.red)
            print("\nMin Special_RRT distance:")
            print(minPathDist)

        return solutionPath, direction, minPathDist, circleCoor

    def solveWithBacktrack2(self, start, goal, backtrackSteps = 5, rep = 2000, map = None, drawAll = False):
        self.reset(start)
        self.goal = goal
        self.straitDist2root = self.distance(self.root, self.goal)
        self.backtrackSteps = backtrackSteps
        if not map == None:
            map.drawMap()

        distStart2goal = 1.01 * self.straitDist2root
        minPathDist = 1000000*distStart2goal
        iter = 0
        while len(self.goalID) == 0 or iter < rep:
            iter += 1
            if iter % 10 == 1:
                childCoor, parentCoor = self.backtrackBias(self.goal)
            else:
                childCoor, parentCoor = self.backtrackExpand()
            if childCoor[0] == self.goal[0] and childCoor[1] == self.goal[1]:
                sol, dist = self.backtrackRefine()
                if dist < minPathDist - self.maxStep/1000:
                    solutionPath, minPathDist = self.backtrackRefine()
                    if not map == None:
                        if drawAll:
                            map.drawMap()
                            map.drawTree(self.x, self.y, self.parentIDs)
                        map.drawPath(solutionPath)
                        print("\nID: {}".format(len(self.goalDist2root)-1))
                        print("Num of nodes: {}".format(self.numNodes))
                        print("Path length: {}".format(self.goalDist2root[len(self.goalID) - 1]))
                if dist < distStart2goal: break
                self.removeAllNodes(start)
            if drawAll and not map == None:
                map.drawCircle(map.grey, childCoor, 2, 0)
                map.drawLine(map.blue, childCoor, parentCoor, 1)

        if len(self.goalID) > 0:
            #solutionPath, noChangeIDs = self.shortenPath(solutionPath)
            #solutionPath = self.removeIDs(solutionPath, noChangeIDs)
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
    from pygameStructure.obstacles import Obstacles
    from pygameStructure.map import Map
    import time as TIME
    import numpy as np

    mapDim = [600, 600]
    start = [50,50]
    goal = [mapDim[0]-50, mapDim[1]-50]
    obsDim = 80
    obsNum = 20

    obsGenerator = Obstacles()
    rrt = RRT(mapDim)
    map = Map(mapDim)
    map.setStartGoal(start, goal)



    iter = 20
    rounds = 4
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
            rep = 1000 + 1000 * j
            rrt.setSolveParam(20, 5, 20, 500)
            solutionPath, dist[j][i] = rrt.solve(start, goal, rep, map, True)
            #solutionPath, direction, dist[j][i] = rrt.solveWithBacktrack(start, goal, 3, rep)#, map, True)
            end = TIME.time()
            time[j][i] = end - begin

            print("time {}: {}".format(j, time[j][i]))
            print("distance {}: {}".format(j, dist[j][i]))
            aveDist[j] += dist[j][i]
            aveTime[j] += time[j][i]
            
            map.drawMap()
            map.drawPath(solutionPath)
            TIME.sleep(1)

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

    pygame.event.clear()
    pygame.event.wait()

if __name__ == '__main__':
    main()