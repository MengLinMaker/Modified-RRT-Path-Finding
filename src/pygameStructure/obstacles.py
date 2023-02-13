import random as rnd
import pygame

class Obstacles:
    def __init__(self, obsList = []):
        self.setObjects(obsList)

    def setObjects(self, obsList = []):
        self.obsList = obsList

    def makeObs(self, mapDim, start, goal, obsDim, obsNum):
        obsList = []
        for i in range(0,obsNum):
            rect = None
            collide = True
            while collide:
                upper = [rnd.uniform(0, mapDim[0] - obsDim),
                         rnd.uniform(0, mapDim[1] - obsDim)]
                rect = pygame.Rect(upper, (obsDim, obsDim))
                if rect.collidepoint(start) or rect.collidepoint(goal):
                    collide = True
                else:
                    collide = False

            obsList.append(rect)
        self.obsList = obsList.copy()
        return obsList

    def makeNoneCollideObs(self, mapDim, start, goal, obsDim, obsNum):
        obsList = []
        for i in range(0,obsNum):
            rect = None
            collide = True
            while collide:
                upper = [int(rnd.uniform(0, mapDim[0] - obsDim)),
                         int(rnd.uniform(0, mapDim[1] - obsDim))]
                rect = pygame.Rect(upper, (obsDim, obsDim))
                if rect.collidepoint(start) or rect.collidepoint(goal):
                    collide = True
                else:
                    collide = False
                    for obs in obsList:
                        if rect.collidepoint(obs.bottomleft) or rect.collidepoint(obs.bottomright) or \
                                rect.collidepoint(obs.topleft) or rect.collidepoint(obs.topright):
                            collide = True
                            break
            obsList.append(rect)
        self.obsList = obsList.copy()
        return obsList