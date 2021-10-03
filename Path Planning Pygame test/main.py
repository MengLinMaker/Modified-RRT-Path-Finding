import pygame
from RRT import RRT
from RRT_star import RRT_star
from Map import Map
from Obstacles import Obstacles
import random as rnd

import numpy as np
import ast
import time


# read in the object poses, note that the object pose array is [y,x]
def parse_map(fname: str) -> dict:
    with open(fname, 'r') as f:
        gt_dict = ast.literal_eval(f.readline())
        apple_gt, lemon_gt, person_gt, aruco_gt = [], [], [], []

        # remove unique id of targets of the same type
        for key in gt_dict:
            if key.startswith('apple'):
                apple_gt.append(np.array(list(gt_dict[key].values()), dtype=float))
            elif key.startswith('lemon'):
                lemon_gt.append(np.array(list(gt_dict[key].values()), dtype=float))
            elif key.startswith('person'):
                person_gt.append(np.array(list(gt_dict[key].values()), dtype=float))
            elif key.startswith('aruco'):
                aruco_gt.append(np.array(list(gt_dict[key].values()), dtype=float))

    # if more than 3 estimations are given for a target type, only the first 3 estimations will be used
    if len(apple_gt) > 3:
        apple_gt = apple_gt[0:3]
    if len(lemon_gt) > 3:
        lemon_gt = lemon_gt[0:3]
    if len(person_gt) > 3:
        person_gt = person_gt[0:3]

    return apple_gt, lemon_gt, person_gt, aruco_gt



def main():
    mapDim = [700,700]
    start = [450,600]
    goal = [mapDim[0]-50, mapDim[1]-50]
    #goal = [360, 180]
    obsDim = 50
    obsNum = 50


    obsGenerator = Obstacles()
    obsList = obsGenerator.makeObs(mapDim, start, goal, obsDim, obsNum)

    pygame.init()


    #'''
    fname = 'TRUEMAP.txt'
    apple_gt, lemon_gt, person_gt, aruco_gt = parse_map(fname)
    mapObstacles = []
    minCoor = [-1.5, -1.5]
    scale = 200

    size = [0.35,0.35]
    size = [scale * size[0], scale * size[1]]
    for coor in apple_gt:
        coor = [scale * (coor[0] - minCoor[0]), scale * (coor[1] - minCoor[1])]
        upper = [coor[0] + 0.5*size[0], coor[1] + 0.5*size[1]]
        obs = pygame.Rect(upper, size)
        mapObstacles.append(obs)
    size = [0.35,0.35]
    size = [scale * size[0], scale * size[1]]
    for coor in lemon_gt:
        coor = [scale * (coor[0] - minCoor[0]), scale * (coor[1] - minCoor[1])]
        upper = [coor[0] + 0.5 * size[0], coor[1] + 0.5 * size[1]]
        obs = pygame.Rect(upper, size)
        mapObstacles.append(obs)
    size = [0.35,0.35]
    size = [scale * size[0], scale * size[1]]
    for coor in person_gt:
        coor = [scale * (coor[0] - minCoor[0]), scale * (coor[1] - minCoor[1])]
        upper = [coor[0] + 0.5 * size[0], coor[1] + 0.5 * size[1]]
        obs = pygame.Rect(upper, size)
        mapObstacles.append(obs)
    size = [0.35,0.35]
    size = [scale * size[0], scale * size[1]]
    for coor in aruco_gt:
        coor = [scale * (coor[0] - minCoor[0]), scale * (coor[1] - minCoor[1])]
        upper = [coor[0] + 0.5 * size[0], coor[1] + 0.5 * size[1]]
        obs = pygame.Rect(upper, size)
        mapObstacles.append(obs)
        
    obsList = mapObstacles
    #'''




    #'''
    dist = 0
    collision = True
    while dist < 580 or collision:
        start = np.array([int(rnd.uniform(0, mapDim[0])),
                          int(rnd.uniform(0, mapDim[1]))])
        goal = np.array([int(rnd.uniform(0, mapDim[0])),
                         int(rnd.uniform(0, mapDim[1]))])
        vect = start - goal
        dist = np.linalg.norm(vect)
        collision = False
        for obs in obsList:
            if obs.collidepoint(start[0], start[1]) or obs.collidepoint(goal[0], goal[1]):
                collision = True
    #'''
    #goal = [540, 100]
    #goal =[240, 450]


    rrt = RRT(mapDim)
    rrt_star = RRT_star(mapDim)
    map = Map(mapDim)



    '''
    buffer = 0.12 # 12cm
    mapDim = [3-2*buffer,3-2*buffer]
    minCoor = [ -mapDim[0]/2, -mapDim[1]/2 ]

    # initialise
    rrt = RRT(mapDim, minCoor)
    # 0.1m step, 5 interpPoints, ignore, path optimisation iterations
    rrt.setObstacles(obsList)
    rrt.setSolveParam(0.1, 5, 30, 500)
    # 5000 iterations should be good enough fro solving
    solutionPath, distance = rrt.solve(start, goal, 5000)
    '''




    rrt.setObstacles(obsList)
    rrt_star.setObstacles(obsList)
    map.setObstacles(obsList)

    map.setStartGoal(start, goal)
    # set maxStep, interpPoints, shortenIter

    aveRRTtime = 0
    aveSRRTtime = 0
    aveRRTdist = 0
    aveSRRTdist = 0
    aveBestDist = 0
    iter = 20
    RRTtime = [0]*iter
    SRRTtime = [0]*iter
    RRTdist = [0]*iter
    SRRTdist = [0]*iter
    bestDist = [0]*iter
    objCoor = [0,0]
    for i in range(iter):

        dist = 0
        collision = True
        while dist < 500 or collision:
            start = np.array([int(rnd.uniform(0, mapDim[0])),
                              int(rnd.uniform(0, mapDim[1]))])
            goal = np.array([int(rnd.uniform(0, mapDim[0])),
                             int(rnd.uniform(0, mapDim[1]))])
            objCoor = np.array([int(rnd.uniform(0, mapDim[0])),
                                int(rnd.uniform(0, mapDim[1]))])
            vect = start - goal
            dist = np.linalg.norm(vect)
            collision = False
            for obs in obsList:
                if obs.collidepoint(start[0], start[1]) or obs.collidepoint(goal[0], goal[1]) or obs.collidepoint(objCoor[0], objCoor[1]):
                    collision = True

        map.setStartGoal(start,goal)
        objCoor = [330,330]

        begin = time.time()
        #'''
        rep = 3000
        rrt.setSolveParam(20, 1, 30, 100)
        RRTsol, RRTdist[i] = rrt.solve(start, goal, rep, map, True)
        RRTsol = rrt.resamplePath(RRTsol, RRTdist[i])
        print(RRTsol)
        rep = 1000
        rrt.setSolveParam(20, 5, 20, 500)
        rrt.setPushObsSize(20)
        #RRTsol, forward, RRTdist[i], circleCoor = rrt.solveWithBacktrack(objCoor, goal, 3, rep, map, True)
        #RRTsol, forward, RRTdist[i] = rrt.solvePushObject(start, objCoor, goal, 2, rep, map, True)
        rep = 500

        #RRTsol, RRTdist[i] = rrt.solveWithBacktrack2(start, goal, 3, rep, map, True)

        #'''
        end = time.time()
        RRTtime[i] = end - begin

        #bestPath, bestDist[i] = rrt.solve(start, goal, 5000)

        begin = time.time()
        rep = 50
        rrt_star.setSolveParam(20, 400, 50, 1, 30, 100)
        #SRRTsol, SRRTdist[i] = rrt_star.solve(start, goal, rep)#, map, True)
        rep = 500
        rrt_star.setSolveParam(20, 400, 50, 1, 30, 100)
        #SRRTsol, SRRTdist[i] = rrt_star.solveWithBacktrack(start, goal, 5, rep)#, map, True)

        end = time.time()
        SRRTtime[i] = end - begin

        map.drawMap()
        map.drawPath(RRTsol, map.blue)
        #map.drawPath(SRRTsol, map.red)
        #map.drawPath(bestPath, map.green)
        time.sleep(5)
        print("\nRRT execution time: {}".format(RRTtime[i]))
        print("RRT min distance: {}".format(RRTdist[i]))
        print("Special_RRT execution time: {}".format(SRRTtime[i]))
        print("Special_RRT min distance: {}".format(SRRTdist[i]))

        print("Best distance: {}".format(bestDist[i]))
        aveRRTdist += RRTdist[i]
        aveSRRTdist += SRRTdist[i]
        aveRRTtime += RRTtime[i]
        aveSRRTtime += SRRTtime[i]
        aveBestDist += bestDist[i]



    print("\n\n\n\n\nOVERALL COMPARISONS: ")
    print("..................")
    print("Ave RRT distance: {}".format(aveRRTdist/iter))
    print("Ave RRT time: {}".format(aveRRTtime/iter))
    print("..................")
    print("Ave SRRT distance: {}".format(aveSRRTdist/iter))
    print("Ave SRRT time: {}".format(aveSRRTtime/iter))
    print("..................")
    print("Ave best distance: {}\n".format(aveBestDist/iter))

    print("RRTtime")
    print("SRRTtime")
    print("RRTdist")
    print("SRRTdist")
    print("BestDist")
    print(RRTtime)
    print(SRRTtime)
    print(RRTdist)
    print(SRRTdist)
    print(bestDist)



    pygame.event.clear()
    pygame.event.wait()

if __name__ == '__main__':
    main()