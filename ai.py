from __future__ import print_function
from heapq import * #Hint: Use heappop and heappush
from queue import Queue
import math 

ACTIONS = [(0,1),(1,0),(0,-1),(-1,0)]
# right, bottom, left, up
class AI:
    def __init__(self, grid, type):
        self.grid = grid
        self.set_type(type)
        self.set_search()

    def set_type(self, type):
        self.final_cost = 0
        self.type = type

    def set_search(self):
        self.final_cost = 0
        self.grid.reset()
        self.finished = False
        self.failed = False
        self.previous = {}

        # Initialization of algorithms goes here
        if self.type == "dfs":
            self.frontier = [self.grid.start]
            self.explored = []
            pass
        elif self.type == "bfs":
            self.frontier = Queue(maxsize=0)
            self.frontier.put(self.grid.start)
            self.explored = []
            pass
        elif self.type == "ucs":
            self.frontier = []
            heappush(self.frontier, (0, self.grid.start[0], self.grid.start[1]))
            self.explored = []
            pass
        elif self.type == "astar":
            self.frontier = []
            heappush(self.frontier, (0, self.grid.start[0], self.grid.start[1], 0))
            self.explored = []
            pass

    def get_result(self):
        total_cost = 0
        current = self.grid.goal
        while not current == self.grid.start:
            total_cost += self.grid.nodes[current].cost()
            current = self.previous[current]
            self.grid.nodes[current].color_in_path = True #This turns the color of the node to red
        total_cost += self.grid.nodes[current].cost()
        self.final_cost = total_cost

    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()

    def dfs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = self.frontier.pop()
        #create direction node for each actions.
        #this DFS uses stack.
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False
        #frontier means surrounding. 
        for n in children:
            #n[0] = x n[1] = y. if (x,y) position is in the grid.
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                #if not puddle
                if not self.grid.nodes[n].puddle and not self.grid.nodes[n].color_checked:
                    #change the previous to current node and then go to next node
                    if n == self.grid.goal:
                        self.previous[n] = current
                        self.finished = True
                    else:
                        if not exist(n, self.explored):
                            self.previous[n] = current
                            self.frontier.append(n)
                            self.explored.append(n)
                            self.grid.nodes[n].color_frontier = True
                        
    
    def bfs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = self.frontier.get()
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False
        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle and not self.grid.nodes[n].color_checked:
                    if n == self.grid.goal:
                        self.previous[n] = current
                        self.finished = True
                    else:
                        if not exist(n, self.explored):
                            self.previous[n] = current
                            self.frontier.append(n)
                            self.explored.append(n)
                            self.grid.nodes[n].color_frontier = True

    
    def ucs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = heappop(self.frontier)
        children = [(current[1]+a[0], current[2]+a[1]) for a in ACTIONS]
        self.grid.nodes[(current[1],current[2])].color_checked = True
        self.grid.nodes[(current[1],current[2])].color_frontier = False
        for n in children:
            #n[0] = x n[1] = y. if (x,y) position is in the grid.
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                #if not puddle
                if not self.grid.nodes[n].puddle and not self.grid.nodes[n].color_checked:
                    if n == self.grid.goal:
                        self.previous[n] = (current[1],current[2])
                        self.finished = True
                    else:
                        new_cost = current[0]+ self.grid.nodes[n].cost()
                        if not exist(n, self.explored):
                            self.previous[n] = (current[1],current[2])
                            heappush(self.frontier, (new_cost, n[0], n[1]))
                            self.explored.append(n)
                            self.grid.nodes[n].color_frontier = True
                        elif lower_cost(n, self.frontier, new_cost):
                            removeNode(self.frontier, n)
                            self.previous[n] = (current[1],current[2])
                            heappush(self.frontier, (new_cost, n[0], n[1]))
                            self.grid.nodes[n].color_frontier = True
    
    #Implement Astar here (Don't forget implement initialization at line 23)
    def astar_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = heappop(self.frontier)
        children = [(current[1]+a[0], current[2]+a[1]) for a in ACTIONS]
        self.grid.nodes[(current[1],current[2])].color_checked = True
        self.grid.nodes[(current[1],current[2])].color_frontier = False
        for n in children:
            #n[0] = x n[1] = y. if (x,y) position is in the grid.
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                #if not puddle
                if not self.grid.nodes[n].puddle and not self.grid.nodes[n].color_checked:
                    if n == self.grid.goal:
                        self.previous[n] = (current[1],current[2])
                        self.finished = True
                    else:
                        new_cost = current[0]+ self.grid.nodes[n].cost() - current[3]
                        h = manDistance(n, self.grid.goal)
                        # h = diagDistance(n, self.grid.goal)
                        # h = eucliDistance(n, self.grid.goal)
                        if not exist(n, self.explored):
                            self.previous[n] = (current[1],current[2])
                            heappush(self.frontier, (new_cost+h, n[0], n[1], h))
                            self.explored.append(n)
                            self.grid.nodes[n].color_frontier = True
                        elif lower_cost(n, self.frontier, new_cost+h) and exist(n, self.explored):
                            removeNode(self.frontier, n)
                            self.previous[n] = (current[1],current[2])
                            heappush(self.frontier, (new_cost+h, n[0], n[1], h))
                            self.grid.nodes[n].color_frontier = True

def exist(n, array):
    for m in array:
        if(n == m):
            return True
    return False

#check if it has already appeared in the frontier,
#if new node has lower cost return true 
#else return false.
def lower_cost(n, array, new_cost):
    for m in array:
        if(n == (m[1],m[2])):
            if new_cost < m[0]:
                return True
    return False

def manDistance(n, goal):
    return abs(n[0]-goal[0]) + abs(n[1]-goal[1])

def eucliDistance(n, goal):
    return math.sqrt( ((n[0]-goal[0])**2) + ((n[1]-goal[1])**2) )

def diagDistance(n,goal):
    return max( abs(n[0]-goal[0]),abs(n[1]-goal[1]) )

def removeNode(array, n):
    for m in array:
        if(n == (m[1],m[2])):
            array.remove(m)
            