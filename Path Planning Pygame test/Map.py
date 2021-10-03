import pygame
import math

class Map:
    def __init__(self,  mapDim, start = [0,0], goal = [0,0], obsList = []):
        self.start = start
        self.goal = goal
        self.mapDim = mapDim
        self.obsList = obsList

        # make pygame window
        self.MapWindowName = 'Path planning algorithms'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode(self.mapDim)

        # sizes
        self.nodeRad = int(2)
        self.nodeThickness = int(0)
        self.edgeThickness = int(1)

        # colours
        self.black = (0,0,0)
        self.grey = (150,150,150)
        self.blue = (0,0,255)
        self.green = (0,255,0)
        self.red = (255,0,0)
        self.white = (255,255,255)

        #self.drawMap()

    ##################################################
    # set parameters
    ##################################################

    def setObstacles(self, obsList = []):
        self.obsList = obsList
        self.drawMap()

    def setStartGoal(self, start, goal):
        self.start = start
        self.goal = goal

    ##################################################
    # drawing methods
    ##################################################

    def drawMap(self):
        self.map.fill(self.white)
        pygame.draw.circle(self.map, self.green, self.start, 5, 0)
        pygame.draw.circle(self.map, self.red, self.goal, 5, 0)
        self.drawObs(self.obsList)
        pygame.display.update()

    def drawPath(self, nodeCoor, colour = [0,0,0]):
        oldCoor = None
        for coor in nodeCoor:
            self.drawCircle(colour, coor, 3, 0)
            if oldCoor != None:
                self.drawLine(colour, coor, oldCoor, 1)
            oldCoor = coor
        pygame.display.update()

    def drawObs(self,obsList):
        obsList = obsList.copy()
        while len(obsList) > 0:
            obs = obsList.pop(0)
            pygame.draw.rect(self.map, self.grey, obs)
            self.drawCircle(self.black, obs.center, math.floor(0.09 * (obs.height + obs.width)), 0)
        pygame.display.update()

    def drawCircle(self, colour, coor, radius, thickness):
        coor = [int(coor[0]), int(coor[1])]
        pygame.draw.circle(self.map, colour, coor, radius, thickness)
        pygame.display.update()

    def drawLine(self, colour, coor1, coor2, thickness):
        coor = [int(coor1[0]), int(coor1[1])]
        coor = [int(coor2[0]), int(coor2[1])]
        pygame.draw.line(self.map, colour, coor1, coor2, thickness)
        pygame.display.update()

    def drawTree(self, x, y, parentIDs):
        list = range(len(x)-1)
        for i in list:
            coor = [x[i],y[i]]
            pID = parentIDs[i]
            parentCoor = [x[pID],y[pID]]
            self.drawCircle(self.grey, coor, 2, 0)
            self.drawLine(self.blue, coor, parentCoor, 1)