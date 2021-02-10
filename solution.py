#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os
from search import * #for search engines
from snowman import SnowmanState, Direction, snowman_goal_state #for snowball specific classes
from test_problems import PROBLEMS #20 test problems

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a snowman state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each snowball that has yet to be stored and the storage point is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.

    t_m_distance = 0   #total manhattan distance
    goal = state.destination
    for snow in state.snowballs:
      stack = state.snowballs[snow]
      distance = abs(snow[0] - goal[0]) + abs(snow[1] - goal[1])

      # check if there is a two snowball stack
      if  (stack == 3) or (stack == 4) or (stack == 5):
            distance *= 2

      # check if there is a three snowball stack
      elif (stack == 6):
            distance *= 3

      t_m_distance += distance
    return t_m_distance

    


#HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible snowball heuristic'''
  '''INPUT: a snowball state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''   
  return len(state.snowballs)

def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a snowball state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.

    inf = float('inf')
    toal = 0   #total heur value
    goal = state.destination
    robot = state.robot
    obs = state.obstacles
    robot_to_snow = []
    for snow in state.snowballs:
      x = snow[0]
      y = snow[1]
      robot_to_snow.append(abs(x - robot[0]) + abs(y - robot[1]))

      # one of the snowball is in the corner of the board
      if ((x == 0 and y == 0) or (x == state.width - 1 and y == state.height - 1) or (x == 0 and y == state.height - 1) or (y == 0 and x == state.width - 1)):
            if (goal != snow):
              return inf

              #if in corner with size 1, 2, 4, 5 then cannot move
            elif (state.snowballs[snow] == 2 or state.snowballs[snow] == 1 or state.snowballs[snow] == 4 or state.snowballs[snow] == 5) and (snow == goal):
              return inf
      
      # one of the snowball is next to the side board while the destination is not on that side
      if ((x == 0 or x == state.width - 1)):
            if (x != goal[0]):
                  return inf
            elif (x,y) != goal and (state.snowballs[snow] == 3 or state.snowballs[snow] == 4 or state.snowballs[snow] == 5 or state.snowballs[snow] == 6):
                  return inf

            
      elif ((y == 0 or y == state.height - 1) and (y != goal[1])):
            return inf

      # there is two obstacles next to the snowball while it is not the destination
      if ((((x+1, y) in obs) and ((x, y+1) in obs)) or (((x+1, y) in obs) and ((x, y-1) in obs)) or (((x, y+1) in obs) and ((x-1, y) in obs)) or (((x, y-1) in obs) and ((x-1, y) in obs))) and (snow != goal):
            return inf
      
      if ((x+1,y) in obs and goal[0] < x) or ((x-1,y) in obs and goal[0] > x):
                toal += 3
      if ((x,y+1) in obs and goal[1] < y) or ((x,y-1) in obs and goal[1] > y):
            toal += 3

      if snow != goal:
            stack = state.snowballs[snow]
            distance = abs(x - goal[0]) + abs(y - goal[1])

            # check if there is a two snowball stack
            if  (stack == 3) or (stack == 4) or (stack == 5):
                  distance *= 2

            # check if there is a three snowball stack
            elif (stack == 6):
                  distance *= 3

            # add manhatten distance to total
            toal += distance
    
    toal += min(robot_to_snow)
    return toal


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    
    return sN.gval + (weight * sN.hval)

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 5):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  goal = False
  # the stop time and curr time
  curr = os.times()[4]
  stop_time = curr + timebound - 0.2
  #initial the search engine
  wrapped_fval_function = (lambda sN: fval_function(sN, weight))
  se = SearchEngine('custom', 'full')
  se.init_search(initial_state, snowman_goal_state, heur_fn, wrapped_fval_function)
  costbound = (float('inf'), float('inf'), float('inf'))   #Since we only continue if g+h < current cost of goal state

  result = se.search(timebound-0.2)

  while curr < stop_time:
          # return the last goal if the iterative search call return false.
          # return False if the first time we call search returns false
        if result == False:
              return goal

        curr = os.times()[4]
        
        timebound = stop_time - curr
        if result.gval < costbound[2]:
              costbound = (float('inf'), float('inf'), result.gval)  # Set g and h value to inf so that we only compare the f value
              goal = result

        
        result = se.search(timebound, costbound)

  return goal

def anytime_gbfs(initial_state, heur_fn, timebound = 5):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  goal = None

  # the stop time and curr time
  curr = os.times()[4]
  stop_time = os.times()[4] + timebound - 0.2

  #initial the search engine
  se = SearchEngine('best_first', 'full')
  se.init_search(initState=initial_state, goal_fn=snowman_goal_state, heur_fn=heur_fn)
  costbound = (float('inf'), float('inf'), float('inf'))   #Since we only continue if g < current cost of goal state
  result = se.search(timebound-0.2)
  
  #return False if the first time we call search returns false
  if result == False:
        return False

  while curr < stop_time:
          # return the last goal if the iterative search call return false.
        if result == False:
              return goal

        curr = os.times()[4] 
        new_timebound = stop_time - curr
        if result.gval < costbound[0]:
              costbound = (result.gval, float('inf'), float('inf'))  # Set g+h and h value to inf so that we only compare the g value
              goal = result
        result = se.search(timebound=new_timebound, costbound=costbound)

  return goal
