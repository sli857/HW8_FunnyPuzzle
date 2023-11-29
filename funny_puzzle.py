import heapq
import copy

# assume 3*3 grid
def getXandY(index):
    return index % 3, int(index / 3)

def getIndex(x, y):
    if(x < 0 or x > 2 or y < 0 or y > 2):
        return -1
    return 3 * y + x

def swap(lst, i, j):
    lst[i], lst[j] = lst[j], lst[i]

def get_manhattan_distance(from_state, to_state=[1, 2, 3, 4, 5, 6, 7, 0, 0]):
    """
    TODO: implement this function. This function will not be tested directly by the grader. 

    INPUT: 
        Two states (if second state is omitted then it is assumed that it is the goal state)

    RETURNS:
        A scalar that is the sum of Manhattan distances for all tiles.
    """
    leng = len(to_state)
    if(len(from_state) != leng):
        raise ValueError("len(from_state) != len(to_state)")
    distance = 0
    for i in range(leng):
        value = to_state[i]
        if(value == 0): 
            continue
        index = from_state.index(value)
        x1, y1 = getXandY(index)
        x2, y2 = getXandY(i)
        distance += abs(x1 - x2) + abs(y1 - y2)
    return distance

def print_succ(state):
    """
    TODO: This is based on get_succ function below, so should implement that function.

    INPUT: 
        A state (list of length 9)

    WHAT IT DOES:
        Prints the list of all the valid successors in the puzzle. 
    """
    succ_states = get_succ(state)

    for succ_state in succ_states:
        print(succ_state, "h={}".format(get_manhattan_distance(succ_state)))


def get_succ(state):
    """
    TODO: implement this function.

    INPUT: 
        A state (list of length 9)

    RETURNS:
        A list of all the valid successors in the puzzle (don't forget to sort the result as done below). 
    """
    leng = len(state)
    if(leng != 9):
        raise ValueError
    succ_states = []
    for i in range(leng):
        if(state[i] == 0):
            continue
        x, y = getXandY(i)
        #print(x, y)
        temp_state = copy.deepcopy(state)

        idx1 = getIndex(x-1, y)
        if(idx1 != -1):
            if(temp_state[idx1] == 0):
                swap(temp_state, i, idx1)
                succ_states.append(temp_state)
                temp_state = copy.deepcopy(state)

        idx2 = getIndex(x+1, y)
        if(idx2 != - 1):
            if(temp_state[idx2] == 0):
                swap(temp_state, i, idx2)
                succ_states.append(temp_state)
                temp_state = copy.deepcopy(state)
        
        idx3 = getIndex(x, y-1)
        if(idx3 != -1):
            if(temp_state[idx3] == 0):
                swap(temp_state, i, idx3)
                succ_states.append(temp_state)
                temp_state = copy.deepcopy(state)

        idx4 = getIndex(x, y+1)
        if(idx4 != -1):
            if(temp_state[idx4] == 0):
                swap(temp_state, i, idx4)
                succ_states.append(temp_state)
    return sorted(succ_states)


def solve(state, goal_state=[1, 2, 3, 4, 5, 6, 7, 0, 0]):
    """
    TODO: Implement the A* algorithm here.

    INPUT: 
        An initial state (list of length 9)

    WHAT IT SHOULD DO:
        Prints a path of configurations from initial state to goal state along  h values, number of moves, and max queue number in the format specified in the pdf.
    """
    pq = []
    state_info_list = []
    initial_dis = get_manhattan_distance(state)
    heapq.heappush(pq, (initial_dis, state, (0, initial_dis, -1)))
    max_queue_length = 1
    while(pq):
        popped = heapq.heappop(pq)
        curr_cost = popped[0]
        curr_state = popped[1]
        curr_info = popped[2]
        
        curr_move = [curr_state, curr_info[1], curr_info[0]]
        state_info_list.append(curr_move)

        succ_list = get_succ(curr_state)
        print(state_info_list)
        for x in succ_list:
            if((x != y[0] for y in state_info_list) or (x != z[1] for z in pq)):
                heapq.heappush(pq, (curr_cost + get_manhattan_distance(x), x, (curr_info[0]+1, get_manhattan_distance(x), state_info_list.index(curr_move))))

        #print(state_info_list)


    # This is a format helper.
    # build "state_info_list", for each "state_info" in the list, it contains "current_state", "h" and "move".
    # define and compute max length
    # it can help to avoid any potential format issue.

    # for state_info in state_info_list:
    #     current_state = state_info[0]
    #     h = state_info[1]
    #     move = state_info[2]
    #     print(current_state, "h={}".format(h), "moves: {}".format(move))
    # print("Max queue length: {}".format(max_length))

if __name__ == "__main__":
    """
    Feel free to write your own test code here to exaime the correctness of your functions. 
    Note that this part will not be graded.
    """
    # print_succ([2,5,1,4,0,6,7,0,3])
    # print()

    # print(get_manhattan_distance([2,5,1,4,0,6,7,0,3], [1, 2, 3, 4, 5, 6, 7, 0, 0]))
    # print()


    solve([4, 3, 0, 5, 1, 6, 7, 2, 0])
    print()
