from collections import deque
from queue import PriorityQueue
from pyamaze import maze,agent,COLOR,textLabel
from timeit import timeit


def manh(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return (abs(x1 - x2) + abs(y1 - y2))

def aStar(m,start=None):
    if start is None:
        start=(m.rows,m.cols)
    open = PriorityQueue()
    open.put((manh(start, m._goal), manh(start, m._goal), start))
    aPath = {}
    g_score = {row: float("inf") for row in m.grid}
    g_score[start] = 0
    f_score = {row: float("inf") for row in m.grid}
    f_score[start] = manh(start, m._goal)
    searchPath=[start]
    while not open.empty():
        currCell = open.get()[2]
        searchPath.append(currCell)
        if currCell == m._goal:
            break        
        for d in 'ESNW':
            if m.maze_map[currCell][d]==True:
                if d=='E':
                    childCell=(currCell[0],currCell[1]+1)
                elif d=='W':
                    childCell=(currCell[0],currCell[1]-1)
                elif d=='N':
                    childCell=(currCell[0]-1,currCell[1])
                elif d=='S':
                    childCell=(currCell[0]+1,currCell[1])
                temp_g_score = g_score[currCell] + 1
                temp_f_score = temp_g_score + manh(childCell, m._goal)
                if temp_f_score < f_score[childCell]: 
                    aPath[childCell] = currCell
                    g_score[childCell] = temp_g_score
                    f_score[childCell] = temp_g_score + manh(childCell, m._goal)
                    open.put((f_score[childCell], manh(childCell, m._goal), childCell))


    fwdPath={}
    cell=m._goal
    while cell!=start:
        fwdPath[aPath[cell]]=cell
        cell=aPath[cell]
    return searchPath,aPath,fwdPath


def ecud(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return ((x1 - x2) **2 + (y1 - y2)**2) ** (0.5)

def aStar2(m,start=None):
    if start is None:
        start=(m.rows,m.cols)
    open = PriorityQueue()
    open.put((ecud(start, m._goal), ecud(start, m._goal), start))
    aPath = {}
    g_score = {row: float("inf") for row in m.grid}
    g_score[start] = 0
    f_score = {row: float("inf") for row in m.grid}
    f_score[start] = ecud(start, m._goal)
    searchPath=[start]
    while not open.empty():
        currCell = open.get()[2]
        searchPath.append(currCell)
        if currCell == m._goal:
            break        
        for d in 'ESNW':
            if m.maze_map[currCell][d]==True:
                if d=='E':
                    childCell=(currCell[0],currCell[1]+1)
                elif d=='W':
                    childCell=(currCell[0],currCell[1]-1)
                elif d=='N':
                    childCell=(currCell[0]-1,currCell[1])
                elif d=='S':
                    childCell=(currCell[0]+1,currCell[1])
                temp_g_score = g_score[currCell] + 1
                temp_f_score = temp_g_score + ecud(childCell, m._goal)
                if temp_f_score < f_score[childCell]: 
                    aPath[childCell] = currCell
                    g_score[childCell] = temp_g_score
                    f_score[childCell] = temp_g_score + ecud(childCell, m._goal)
                    open.put((f_score[childCell], ecud(childCell, m._goal), childCell))
    fwdPath={}
    cell=m._goal
    while cell!=start:
        fwdPath[aPath[cell]]=cell
        cell=aPath[cell]
    return searchPath,aPath,fwdPath



myMaze=maze(50,70)
myMaze.CreateMaze(loopPercent=70)
searchPath,aPath,fwdPath=aStar(myMaze)
bSearch,bfsPath,fwdBFSPath=aStar2(myMaze)

l=textLabel(myMaze,'A-Star (M) Path Length',len(fwdPath)+1)
l=textLabel(myMaze,'A-Star (E) Path Length',len(fwdBFSPath)+1)
l=textLabel(myMaze,'A-Star (M) Search Length',len(searchPath)+1)
l=textLabel(myMaze,'A-Star (E) Length',len(bSearch)+1)

a=agent(myMaze,footprints=True,color=COLOR.cyan,filled=True)
b=agent(myMaze,footprints=True,color=COLOR.yellow)
myMaze.tracePath({a:fwdBFSPath},delay=50)
myMaze.tracePath({b:fwdPath},delay=50)

t1=timeit(stmt='aStar(myMaze)',number=10,globals=globals())
t2=timeit(stmt='aStar2(myMaze)',number=10,globals=globals())

textLabel(myMaze,'A-Star (M) Time',t1)
textLabel(myMaze,'A-Star (E) Time',t2)


myMaze.run()