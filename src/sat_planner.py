from z3 import *

def plan_sat(solver, RobotAt, BoxAt, max_t, grid_size, num_boxes, grid):
    set_param("parallel.enable", True)          #
    set_param("smt.threads", 8)
    check_result = solver.check()
    if check_result != sat:
        return None

    model = solver.model()
    rows, cols = grid_size
    path = []
    box_paths = [[] for _ in range(num_boxes)]

    for t in range(max_t + 1):
        # Robot position
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] == '#':          
                    continue
                if model.evaluate(RobotAt[(t, r, c)], model_completion=True):
                    path.append((r, c))
                    break

        # Box positions
        for b_id in range(num_boxes):
            for r in range(rows):
                for c in range(cols):
                    if grid[r][c] == '#':      
                        continue
                    if model.evaluate(BoxAt[(t, r, c, b_id)], model_completion=True):
                        box_paths[b_id].append((r, c))
                        break

    return path, box_paths
