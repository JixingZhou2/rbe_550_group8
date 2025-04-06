from sat_encoding import encode_sat_plan
from sat_planner import plan_sat
from visualize import visualize_path

def load_map(filename):
    with open(filename, 'r') as f:
        return [list(line.strip()) for line in f.readlines()]


map_name = "icepath.txt"
grid = load_map(f"../maps/{map_name}")


start = (0, 0)
goal = (0, 0)
boxes = []  
obstacles = [(x,y) for x,row in enumerate(grid) for y,cell in enumerate(row) if cell == '#']

max_t = 10
solver, RobotAt = encode_sat_plan(grid, start, goal, boxes, obstacles, max_t)
path = plan_sat(solver, RobotAt, max_t, (len(grid), len(grid[0])))

if path:
    visualize_path(grid, path)
else:
    print("No solution found.")