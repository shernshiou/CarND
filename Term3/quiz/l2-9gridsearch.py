# ----------
# User Instructions:
#
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):
    opened = [[0, init[0], init[1]]]
    blocked = [[False if cell == 0 else True for cell in row] for row in grid]

    while opened:
        # Take list item
        # print('take list item:')
        opened.sort()
        curr = opened.pop(0)
        # print(curr)

        if curr[1] == goal[0] and curr[2] == goal[1]:
            return curr

        blocked[curr[1]][curr[2]] = True
        # New opened list
        moves = list_moves(grid, blocked, [curr[1], curr[2]])
        for move in moves:
            opened.append([curr[0]+cost, move[0], move[1]])
        # print('new open list:')
        # for coor in opened:
        #     print(coor)


    return 'fail'

def list_moves(grid, blocked, curr):
    moves = []
    for direction in delta:
        x = curr[0] + direction[0]
        y = curr[1] + direction[1]
        if x >= 0 and x < len(grid) and y >= 0 and y < len(grid[0]):
            if not blocked[x][y]:
                moves.append([x, y])
    return moves

print(search(grid,init,goal,cost))
