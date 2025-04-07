from z3 import sat

def plan_sat(solver, RobotAt, BoxAt, max_t, grid_size, num_boxes):
    """
    Solves the SAT instance and extracts both the robot and box positions over time.

    :param solver:     A Z3 solver with constraints added.
    :param RobotAt:    Dict (t, r, c) -> Bool variable for robot location.
    :param BoxAt:      Dict (t, r, c, b_id) -> Bool variable for box location.
    :param max_t:      Max time step.
    :param grid_size:  (rows, cols)
    :param num_boxes:  Number of boxes
    :return:           (robot_path, box_paths) if SAT, else None
    """
    check_result = solver.check()
    if check_result != sat:
        return None

    model = solver.model()
    path = []
    box_paths = [[] for _ in range(num_boxes)]
    rows, cols = grid_size

    for t in range(max_t + 1):
        # Robot position
        for r in range(rows):
            for c in range(cols):
                if model.evaluate(RobotAt[(t, r, c)], model_completion=True):
                    if model.evaluate(RobotAt[(t, r, c)]):
                        path.append((r, c))
                        break

        # Box positions
        for b_id in range(num_boxes):
            for r in range(rows):
                for c in range(cols):
                    if model.evaluate(BoxAt[(t, r, c, b_id)], model_completion=True):
                        if model.evaluate(BoxAt[(t, r, c, b_id)]):
                            box_paths[b_id].append((r, c))
                            break

    return path, box_paths
