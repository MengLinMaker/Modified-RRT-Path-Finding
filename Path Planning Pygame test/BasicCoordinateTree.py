class BasicCoordinateTree:
    def __init__(self, root):
        self.reset(root)

    def reset(self, root):
        self.root = root
        self.x = [root[0]]
        self.y = [root[1]]
        self.parentIDs = [None]
        self.dist2root = [0]
        self.numNodes = 1

    ##################################################
    # fundamental methods
    ##################################################

    def addNode(self, coor, nodeID):
        self.x.insert(nodeID, coor[0])
        self.y.insert(nodeID, coor[1])
        self.numNodes += 1

    def removeNode(self, nodeID):
        for i in range(self.numNodes):
            if self.parentIDs[i] > nodeID: self.parentIDs[i] -= 1
        self.x.pop(nodeID)
        self.y.pop(nodeID)
        self.parentIDs.pop(nodeID)
        self.dist2root.pop(nodeID)
        self.numNodes -= 1

    def addEdge(self, childID, parentID):
        self.parentIDs.insert(childID, parentID)
        parentValue = self.dist2root[parentID]
        dist = parentValue + self.distanceID(childID, parentID)
        self.dist2root.insert(childID, dist)

    def removeEdge(self, childID):
        self.parentIDs.pop(childID)
        self.dist2root.pop(childID)

    def numOfNodes(self):
        return self.numNodes

    def getCoor(self, nodeID):
        coor = [self.x[nodeID], self.y[nodeID]]
        return coor

    def getDist2root(self, nodeID):
        return self.dist2root[nodeID]

    ##################################################
    # distance methods
    ##################################################

    def distance(self, coor1, coor2):
        px = (coor1[0] - coor2[0]) ** 2
        py = (coor1[1] - coor2[1]) ** 2
        return (px + py) ** 0.5

    def distanceID(self, nodeID1, nodeID2):
        coor1 = self.getCoor(nodeID1)
        coor2 = self.getCoor(nodeID2)
        return self.distance(coor1, coor2)

    def dist2(self, coor1, coor2):
        px = (coor1[0] - coor2[0]) ** 2
        py = (coor1[1] - coor2[1]) ** 2
        return px + py

    def dist2ID(self, nodeID1, nodeID2):
        coor1 = self.getCoor(nodeID1)
        coor2 = self.getCoor(nodeID2)
        return self.dist2(coor1, coor2)

    def norm2(self, coor):
        return coor[0]**2 + coor[1]**2

    ##################################################
    # path creation methods
    ##################################################

    def path2rootID(self, nodeID):
        pathID = []
        pathID.append(nodeID)
        newpos = self.parentIDs[nodeID]
        while newpos != 0:
            pathID = [newpos] + pathID
            newpos = self.parentIDs[newpos]
        pathID.insert(0,0)
        return pathID

    def path2root(self, endID):
        pathCoor = []
        for nodeID in self.path2rootID(endID):
            coor = [self.x[nodeID], self.y[nodeID]]
            pathCoor.append(coor)
        return pathCoor