from z3 import *

def plan_sat(planner, RobotAt, max_t, grid_shape):
    if planner.check() == sat:
        m = planner.model()
        path = []
        return path
    else:
        return None
