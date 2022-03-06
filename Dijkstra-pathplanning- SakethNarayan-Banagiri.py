import cv2 as cv
import numpy as np
import cProfile

# Vistualizing the workspace

space = np.zeros((250,400,3), np.uint8)
pts_hex = np.array([[235, 129.79], [235, 170.20], [200, 190.41],[165, 170.20], [165, 129.79], [200, 109.58]], np.int32)
pts_irr_shape = np.array([[31,65], [115,35], [85,70], [105,155]], np.int32)


cv.circle(space, (300, 65), 40, (255, 255, 0), thickness = -1)


cv.fillPoly(space, [pts_hex],(255,255,0))
cv.fillPoly(space, [pts_irr_shape], (255,255,0))


# Defining obstacles

hex = np.array([[235, 80], [235, 120], [200, 140],
                [165, 120], [165, 80], [200, 60], [235, 80]], np.int32)
hex_center = np.array([200, 100])
triangle_1 = np.array([[31,105], [105,95], [85,180], [31,105]])

triangle_1_center = np.mean(triangle_1[:-1], axis = 0)

triangle_2 = np.array([[31,105], [115,215], [85,180], [31,105]])

triangle_2_center = np.mean(triangle_2[:-1], axis = 0)


def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)


def obstacle(pt):
    
    x, y = pt
    if np.sqrt((x - 300)*2 + (y - 185)*2) <= 45 :
        return True
    
    ret = False
    
    for i in range(len(hex) - 1):
        ret = ret or intersect(pt, hex_center, hex[i], hex[i+1])
    if not ret: 
        return True
    
    for i in range(len(triangle_1) - 1):
        ret = ret or intersect(pt, triangle_1_center, triangle_1[i], triangle_1[i+1])
    if not ret: 
        return True
    
    
    for i in range(len(triangle_2) - 1):
        ret = ret or intersect(pt, triangle_2_center, triangle_2[i], triangle_2[i+1])
    if not ret: 
        return True    
    return False        




#Defining actions:



def index (pos):    # cartesian index
    i = pos[0]
    i = int(i)
    j = pos[1]
    j = int(j)
    return i, j


def left (pos):
    i, j = index(pos)
    cost = 1
    if i > 0 and not (obstacle(pos) ):
        new_pos = (i - 1 , j)
        return new_pos, cost

    else:
        return None

def right (pos):
      i, j = index(pos)
      cost = 1
      if i < 399 and not (obstacle(pos) ):
          new_pos = (i + 1 , j)
          return new_pos, cost

      else:
          return None


def up (pos):
      i, j = index(pos)
      cost = 1
      if j > 0 and not (obstacle(pos) ):
          new_pos = (i  , j - 1)
          return new_pos, cost

      else:
          return None

def down (pos):
      i, j = index(pos)
      cost = 1
      if j < 249 and not (obstacle(pos) ):
          new_pos = (i, j + 1)
          return new_pos, cost

      else:
          return None


def up_left (pos):
      i, j = index(pos)
      cost = 1.4
      if i > 0 and j > 0 and not (obstacle(pos) ):
          new_pos = (i - 1, j - 1)
          return new_pos, cost

      else:
          return None


def up_right (pos):
      i, j = index(pos)
      cost = 1.4
      if i < 399 and j > 0 and not (obstacle(pos) ):
          new_pos = (i + 1, j - 1)
          return new_pos, cost

      else:
          return None

def down_left (pos):
      i, j = index(pos)
      cost = 1.4
      if i > 0 and j < 249 and not (obstacle(pos) ):
          new_pos = (i - 1, j + 1)
          return new_pos, cost

      else:
          return None

def down_right (pos):
      i, j = index(pos)
      cost = 1.4
      if i < 399 and j < 249 and not (obstacle(pos) ):
          new_pos = (i + 1, j + 1)
          return new_pos, cost
      else:
          return None

#Determining the order of the direction by using 'move variable'

def actions(pos, move):
      if move == 0:
          return left(pos)

      elif move == 1:
          return right(pos)

      elif move == 2:
          return up(pos)
      
      elif move == 3:
          return down(pos)

      elif move == 4:
          return up_left(pos)
      
      elif move == 5:
          return up_right(pos)
      
      elif move == 6:
          return down_left(pos)
      
      elif move == 7:
          return down_right(pos)

def init_nodes(start_pos):

  x_grid = np.arange(400)
  y_grid = np.arange(250)
  open_dict = {}

  for x in x_grid:
    for y in y_grid:    
      node_pos = (x, y)

      if node_pos != start_pos:
        open_dict[node_pos] = node(node_pos, None, np.inf)
      else:
        open_dict[node_pos] = node(node_pos, None, 0)

  return open_dict

def find_min_node(open_dict):
    min_pos = min(open_dict, key = lambda k : open_dict[k].c2c)
    return min_pos

class node():
    
  def __init__(self, current, parent, c2c):
      self.parent = parent
      self.current = current
      self.c2c = c2c

  def update_cost(self, new_cost, new_parent):
      if new_cost < self.c2c:
          self.c2c = new_cost
          self.parent = new_parent



#Dijkstra Algorithm:

def Dijkstra ():
    
    start_pos_x = int(input("Enter the x-cordinate of the start:"))
    start_pos_y = int(input("Enter the y-cordinate of the start:"))
    goal_pos_x = int(input("Enter the x-cordinate of the goal:"))
    goal_pos_y = int(input("Enter the y-cordinate of the goal:"))
    start_pos = (start_pos_x, start_pos_y)
    goal_pos = (goal_pos_x, goal_pos_y)

    open_dict = init_nodes(start_pos)
    closed_lis = {start_pos}
    explored = [start_pos]
    
    while len(open_dict):
        min_pos = find_min_node(open_dict)
        closed_lis.add(min_pos)
        print(min_pos)
        # import pdb; pdb.set_trace()
        min_node = open_dict.pop(min_pos)
        explored.append(min_pos)
        for moves in range(8):
            act_output = actions(min_node.current, moves)
            if act_output is not None:
                new_pos, cost = act_output
                # import pdb; pdb.set_trace()
                if new_pos not in closed_lis:
                    # import pdb; pdb.set_trace()
                    open_dict[new_pos].update_cost(cost+min_node.c2c, min_node)     #call by reference 
                    
                    if new_pos == goal_pos:
                        print("solution found")
                        return backtrack(open_dict[new_pos]), explored
    
    print("No Solution")
    return None

def backtrack(node):
    print("Tracking Back")
    p = []
    p.append(node.parent.current)
    parent = node.parent
    if parent is None:
        return p
    while parent is not None:
        p.append(parent.current)
        parent = parent.parent
    p.reverse()
    print(p)
    return p


p , explored= Dijkstra()

def visualize(path, explored):
    ''' Visualise the exploration and the recently found path
    '''
    img = space
    h, w, _ = img.shape
    out = cv.VideoWriter('outpy.mp4',cv.VideoWriter_fourcc(*'mp4v'), 60.0, (w, h))
    
    for i in range(len(explored)):
        pos = (249 - explored[i][1], explored[i][0])
        img[pos] = [0, 255, 0]
        if i%100 == 0:
            out.write(img)
            cv.imshow('output', img)
            cv.waitKey(1)
    for pos in path:
        pos = (249 - pos[1], pos[0])
        img[pos] = [0, 0, 255]
    for i in range(50): 
        out.write(img)
    out.release()
    cv.imshow('output', img)
    cv.waitKey(0)

visualize(p, explored)

