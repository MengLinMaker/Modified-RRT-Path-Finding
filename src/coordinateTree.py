class CoordinateTree:
    def __init__(self, root):
        self.root = root
        self.x = [root[0]]
        self.y = [root[1]]
        self.parentIDs = [0]
        self.childIDs = [[]]
        self.dist2root = [0]
        self.numNodes = 1

    def reset(self, root):
        self.root = root
        self.x = [root[0]]
        self.y = [root[1]]
        self.parentIDs = [0]
        self.childIDs = [[]]
        self.dist2root = [0]
        self.numNodes = 1

    ##################################################
    # fundamental methods
    ##################################################

    def addNode(self, coor, nodeID = None):
        if nodeID == None:
            nodeID = self.numNodes
        self.x.insert(nodeID, coor[0])
        self.y.insert(nodeID, coor[1])
        self.parentIDs.insert(nodeID, None)
        self.childIDs.insert(nodeID, [])
        self.dist2root.insert(nodeID, None)
        self.numNodes += 1

    def removeNode(self, nodeID = None):
        if nodeID == None:
            nodeID = self.numNodes-1
        for i in range(self.numNodes):
            if self.parentIDs[i] > nodeID: self.parentIDs[i] -= 1
            for j in range(len(self.childIDs[i])):
                if self.childIDs[i][j] > nodeID: self.childIDs[i][j] -= 1
        self.removeEdge(nodeID)
        self.x.pop(nodeID)
        self.y.pop(nodeID)
        self.parentIDs.pop(nodeID)
        self.childIDs.pop(nodeID)
        self.dist2root.pop(nodeID)
        self.numNodes -= 1

    def addEdge(self, childID, parentID):
        self.parentIDs[childID] = parentID
        self.childIDs[parentID].append(childID)
        parentValue = self.dist2root[parentID]
        if parentValue != None:
            dist = parentValue + self.distanceID(childID, parentID)
        else: dist = None
        self.dist2root[childID] = dist

    def removeEdge(self, childID = None):
        parentID = self.parentIDs[childID]
        ind = None
        childIDs = self.childIDs[parentID]
        for i in range(len(childIDs)-1):
            if childID == i: ind = i
        if ind != None: self.childIDs[parentID].pop(ind)
        self.parentIDs[childID] = None
        self.dist2root[childID] = None

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

##################################################
# testing
##################################################

def main():
    tree = CoordinateTree([0,0])
    print(tree.parentIDs)
    print(tree.childIDs)
    print(tree.dist2root)
    print(" ")

    tree.addNode([0, 1])
    tree.addNode([0, 2])
    tree.addNode([0, 3])
    tree.addNode([0, 4])
    print(tree.parentIDs)
    print(tree.childIDs)
    print(tree.dist2root)
    print(" ")

    tree.addEdge(1, 0)
    tree.addEdge(4, 0)
    tree.addEdge(3, 2)
    print(tree.parentIDs)
    print(tree.childIDs)
    print(tree.dist2root)
    print(" ")


if __name__ == '__main__':
    main()