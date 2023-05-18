# 8puzzle game using A* with misplaced tiles heuristic, manhattan distance heuristic, and uniform cost search 

from queue import PriorityQueue

goal=[1,2,3,4,5,6,7,8,0]

#take input from user
def take_input():
    print("Enter the initial state of the puzzle in a single line with space separated values")
    initial_state=list(map(int,input().split()))
    return initial_state

#find the position of the blank tile
def find_blank_tile(state):
    for i in range(9):
        if state[i]==0:
            return i

#find the possible moves from the current state
def find_possible_moves(state):
    blank_tile=find_blank_tile(state)
    possible_moves=[]
    if blank_tile==0:
        possible_moves.append(1)
        possible_moves.append(3)
    elif blank_tile==1:
        possible_moves.append(0)
        possible_moves.append(2)
        possible_moves.append(4)
    elif blank_tile==2:
        possible_moves.append(1)
        possible_moves.append(5)
    elif blank_tile==3:
        possible_moves.append(0)
        possible_moves.append(4)
        possible_moves.append(6)
    elif blank_tile==4:
        possible_moves.append(1)
        possible_moves.append(3)
        possible_moves.append(5)
        possible_moves.append(7)
    elif blank_tile==5:
        possible_moves.append(2)
        possible_moves.append(4)
        possible_moves.append(8)
    elif blank_tile==6:
        possible_moves.append(3)
        possible_moves.append(7)
    elif blank_tile==7:
        possible_moves.append(4)
        possible_moves.append(6)
        possible_moves.append(8)
    elif blank_tile==8:
        possible_moves.append(5)
        possible_moves.append(7)
    
    next_states=[]
    for move in possible_moves:
        next_state=state.copy()
        next_state[blank_tile]=next_state[move]
        next_state[move]=0
        next_states.append(next_state)

    return next_states


#misplaced tiles heuristic
def misplaced_tiles_heuristic(state):
    misplaced_tiles=0
    for i in range(9):
        if state[i]!=goal[i]:
            misplaced_tiles+=1
    return misplaced_tiles

#manhattan distance heuristic
def manhattan_distance_heuristic(state):
    manhattan_distance=0
    for i in range(9):
        if state[i]!=0:
            manhattan_distance+=abs(i//3-(state[i]-1)//3)+abs(i%3-(state[i]-1)%3)
    return manhattan_distance

#uniform cost search
def uniform_cost_search(initial_state):
    return 0



#check if the current state is valid state
def is_valid_state(state):
    if len(state)!=9:
        return False
    for i in range(9):
        if state[i]<0 or state[i]>8:
            return False
    return True

#Tree node class
class Node:
    def __init__(self,state,parent,depth,heuristic):
        if heuristic=="misplaced_tiles":
            self.heuristic=misplaced_tiles_heuristic(state)
        elif heuristic=="manhattan_distance":
            self.heuristic=manhattan_distance_heuristic(state)
        elif heuristic=="uniform_cost_search":
            self.heuristic=uniform_cost_search(state)

        self.state=state
        self.parent=parent
        self.depth=depth
        self.cost = self.depth + self.heuristic

    #comparision function for priority queue as we get error while comparing two nodes if there is a tie in the cost
    def __lt__(self,other):
        return self.cost<other.cost
    
    #printing in the form of a matrix
    def __str__(self):
        return str(self.state[:3])+"\n"+str(self.state[3:6])+"\n"+str(self.state[6:9])+"\n"
    
    

# A* search  
def a_star_search(initial_state,heuristic):
    if not is_valid_state(initial_state):
        print("Invalid initial state")
        return
    

    queue=PriorityQueue()
    # set to track already visited nodes
    explored=set()
    queue.put((0,Node(initial_state,None,0,heuristic)))
    while not queue.empty():
        node=queue.get()[1]
        explored.add(tuple(node.state))
        if node.state==goal:
            print("Goal state reached")
            print("Total number of nodes explored:",len(explored))
            print("Depth of the goal state:",node.depth)

            path=[]
            while node.parent!=None:
                path.append(node)
                node=node.parent
            path.append(node)
            path=path[::-1]
            print("Path from initial state to goal state:")
            for pnode in path:
                print("Depth:",pnode.depth)
                print(pnode)
            return path
        
        next_states=find_possible_moves(node.state)
        for next_state in next_states:
            if tuple(next_state) not in explored:
                n_node=Node(next_state,node,node.depth+1,heuristic)
                queue.put((n_node.cost,n_node))
    print("Goal state not reachable")
    return None


#main function
if __name__=="__main__":
    print("Do you want to \n 1) enter a custom initial state or \n 2) use from the existing initial states?")
    choice=int(input())
    if choice==1:
        print("Enter the initial state")
        initial_state=take_input()
    elif choice==2:
        state1=[1,2,3,4,5,6,7,8,0]
        state2=[1,2,3,4,5,6,0,7,8]
        state3=[1,2,3,5,0,6,4,7,8]
        state4=[1,3,6,5,0,2,4,7,8]
        state5=[1,3,6,5,0,7,4,8,2]
        state6=[1,6,7,5,0,3,4,8,2]
        state7=[7,1,2,4,8,5,6,3,0]
        state8=[0,7,2,4,6,1,3,5,8]
        states=[state1,state2,state3,state4,state5,state6,state7,state8]
        depths=[0,2,4,8,12,16,20,24]
        print("Choose any initial state from 1 to 8")
        for i in range(len(states)):
            print("Initial state "+str(i+1)+":"+"at depth "+str(depths[i]))
            print(str(states[i][:3])+"\n"+str(states[i][3:6])+"\n"+str(states[i][6:9])+"\n")
        choice=int(input())
        initial_state=states[choice-1]


    
        

    print("Enter the heuristic:")
    print("1. Misplaced tiles heuristic")
    print("2. Manhattan distance heuristic")
    print("3. Uniform cost search")
    heuristic=int(input())
    if heuristic==1:
        a_star_search(initial_state,"misplaced_tiles")
    elif heuristic==2:
        a_star_search(initial_state,"manhattan_distance")
    elif heuristic==3:
        a_star_search(initial_state,"uniform_cost_search")
    else:
        print("Invalid heuristic")

