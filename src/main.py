# main_mp.py  ------------------------------------------------------
import concurrent.futures as fut
import os, gc
from sat_encoding import encode_sat_plan
from sat_planner   import plan_sat
from visualize     import visualize_path


def load_map(fname):
    with open(fname, "r") as f:
        return [list(line.rstrip("\n")) for line in f]


def find_positions(grid):
    start = goal = None
    obstacles, boxes = [], []
    for r in range(len(grid)):
        for c in range(len(grid[0])):
            ch = grid[r][c]
            if   ch == "S": start = (r, c)
            elif ch == "G": goal  = (r, c)
            elif ch == "#": obstacles.append((r, c))
            elif ch == "B": boxes.append((r, c))
    return (start or (0, 0)), (goal or (0, 0)), obstacles, boxes


# -----------------------------------------------
def worker(t, grid, start, goal, boxes, obstacles):
    solver, RobotAt, BoxAt = encode_sat_plan(
        grid, start, goal, boxes, obstacles, t
    )
    res = plan_sat(solver, RobotAt, BoxAt, t,
                   (len(grid), len(grid[0])), len(boxes))
    if res:
        robot_path, box_paths = res
        return t, robot_path, box_paths
    return None


def main():
    grid = load_map("../maps/icepath.txt")
    start, goal, obstacles, boxes = find_positions(grid)

    MAX_T     = 32
    N_WORKERS = 8 #min(os.cpu_count() or 4, MAX_T)

    with fut.ProcessPoolExecutor(max_workers=N_WORKERS) as ex:
        fut2t = {ex.submit(worker, t, grid, start, goal, boxes, obstacles): t
                 for t in range(MAX_T)}

        for f in fut.as_completed(fut2t):
            result = f.result()
            if result is None:            
                continue

            t_found, robot_path, box_paths = result
            print(f"Solution found! horizon = {t_found}")

            ex.shutdown(wait=False, cancel_futures=True)

            with open("path.txt", "w") as fp:
                fp.write(str(robot_path))
            visualize_path(grid, robot_path, box_paths)
            break
        else:
            print("No solution within horizon", MAX_T)

    gc.collect()


if __name__ == "__main__":
    main()


# from sat_encoding import encode_sat_plan
# from sat_planner import plan_sat
# from visualize import visualize_path

# def load_map(filename):
#     with open(filename, 'r') as f:
#         return [list(line.rstrip('\n')) for line in f]

# def find_positions(grid):
#     start = goal = None
#     obstacles = []
#     boxes = []

#     for r in range(len(grid)):
#         for c in range(len(grid[0])):
#             ch = grid[r][c]
#             if ch == 'S':
#                 start = (r, c)
#             elif ch == 'G':
#                 goal = (r, c)
#             elif ch == '#':
#                 obstacles.append((r, c))
#             elif ch == 'B':
#                 boxes.append((r, c))

#     if not start:
#         start = (0, 0)
#     if not goal:
#         goal = (0, 0)
#     return start, goal, obstacles, boxes

# def main():
#     map_name = "icepath.txt"  # or any other file
#     grid = load_map(f"../maps/{map_name}")

#     start, goal, obstacles, boxes = find_positions(grid)
#     max_t = 50  # Increase if needed
#     for time in range(max_t):
#         solver, RobotAt, BoxAt = encode_sat_plan(grid, start, goal, boxes, obstacles, time)

#         result = plan_sat(solver, RobotAt, BoxAt, time, (len(grid), len(grid[0])), len(boxes))

#         if result:
#             robot_path, box_paths = result
#             print("Solution path found!")
#             with open("path.txt", "w") as f:
#                 f.write(str(robot_path))
#             visualize_path(grid, robot_path, box_paths)
#             break
#         else:
#             print(f"No solution found in time {time}.")

# if __name__ == "__main__":
#     main()