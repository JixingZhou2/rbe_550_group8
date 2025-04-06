from z3 import sat

def plan_sat(solver, RobotAt, max_t, grid_size):
    """
    Solves the SAT instance created by encode_sat_plan and, if satisfiable,
    reconstructs the path of the robot from time 0 to max_t.

    :param solver:     A Z3 solver that already has all constraints added.
    :param RobotAt:    Dict keyed by (t, r, c) -> z3 Bool variable
    :param max_t:      The time horizon used in the encoding
    :param grid_size:  (rows, cols) of the map
    :return:           List of (row, col) for each time step if SAT, else None
    """
    check_result = solver.check()
    if check_result != sat:
        return None

    model = solver.model()
    path = []
    rows, cols = grid_size

    # For each time from 0..max_t, find the unique (r,c) that is True in the model
    for t in range(max_t + 1):
        for r in range(rows):
            for c in range(cols):
                if model.evaluate(RobotAt[(t, r, c)]) == True:
                    path.append((r, c))
                    break  # move to next time once we found the true (r,c)

    return path
