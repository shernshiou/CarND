# ----------
# User Instructions:
#
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal.
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def compute_value(grid,goal,cost):
    value = [[99 for col in range(len(grid[0]))] for row in range(len(grid))]
    expansion = [[0, goal]]

    while expansion:
        expansion.sort()
        curr = expansion.pop(0)
        score = curr[0]
        x = curr[1][0]
        y = curr[1][1]
        grid[x][y] = 1
        value[x][y] = score

        moves = list_moves(grid, [x, y], score+cost)
        expansion = expansion + moves

    return value

def list_moves(grid, curr, score):
    moves = []
    for direction in delta:
        x = curr[0] + direction[0]
        y = curr[1] + direction[1]
        if x >= 0 and x < len(grid) and y >= 0 and y < len(grid[0]):
            if grid[x][y] < 1:
                moves.append([score, [x, y]])
    return moves

value = compute_value(grid, goal, cost)
for i in range(len(value)):
    print(value[i])
