from sat_encoding import encode_sat_plan
from sat_planner import plan_sat
from visualize import visualize_path

def load_map(filename):
    """
    Loads the text file into a 2D list of characters.
    Each row is a list of single-character strings.
    """
    with open(filename, 'r') as f:
        return [list(line.rstrip('\n')) for line in f]

def find_positions(grid):
    """
    Searches the grid for:
      - 'S' = start
      - 'G' = goal
      - '#' = obstacles
    Returns (start, goal, obstacles)
    """
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    start = None
    goal = None
    obstacles = []

    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 'S':
                start = (r, c)
            elif grid[r][c] == 'G':
                goal = (r, c)
            elif grid[r][c] == '#':
                obstacles.append((r, c))

    # If you prefer to default to (0,0) in case S or G is not found:
    if not start:
        start = (0, 0)
    if not goal:
        goal = (0, 0)

    return start, goal, obstacles

def main():
    map_name = "icepath.txt"   # or any map file you have
    grid = load_map(f"../maps/{map_name}")

    # Extract start, goal, and obstacles from the grid
    start, goal, obstacles = find_positions(grid)
    # No boxes in the basic "Ice Path" scenario
    boxes = []

    # Define a time horizon for the SAT planner
    max_t = 5

    # Build the SAT solver and encode constraints
    solver, RobotAt = encode_sat_plan(grid, start, goal, boxes, obstacles, max_t)

    # Attempt to solve
    path = plan_sat(solver, RobotAt, max_t, (len(grid), len(grid[0])))

    if path:
        print("Solution path found!")
        with open("path.txt", "w") as f: f.write(str(path))
        visualize_path(grid, path)
    else:
        print("No solution found.")

if __name__ == "__main__":
    main()
