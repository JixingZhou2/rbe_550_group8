from z3 import Solver, Bool, Implies, And, Or, Not

def encode_sat_plan(grid, start, goal, boxes, obstacles, max_t):
    """
    Encodes the "Ice Path" (no box) domain as a SAT problem using Z3.
    
    :param grid:      2D list of characters, e.g. '.' for free cells, '#' for obstacles, etc.
    :param start:     (row, col) tuple for the robot's start location
    :param goal:      (row, col) tuple for the robot's goal location
    :param boxes:     (Unused here, but kept for consistent interface.)
    :param obstacles: List of (row, col) positions of obstacles. (Also derivable from grid, but shown for clarity.)
    :param max_t:     Maximum time-horizon (plan length)
    :return:          (solver, RobotAt)
                     solver   - the Z3 solver with all constraints added
                     RobotAt  - dictionary keyed by (t, r, c) -> z3 Bool variable
    """
    solver = Solver()
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0

    #
    # 1. Create Boolean variables RobotAt(t, r, c)
    #
    RobotAt = {}
    for t in range(max_t + 1):
        for r in range(rows):
            for c in range(cols):
                RobotAt[(t, r, c)] = Bool(f"RobotAt_{t}_{r}_{c}")

    #
    # 2. Add constraints so that at each time t:
    #    - The robot is in exactly one free (non-obstacle) cell.
    #    - It cannot occupy an obstacle.
    #
    for t in range(max_t + 1):
        # The robot must be in at least one free cell
        possible_positions = []
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] != '#':  # Not an obstacle
                    possible_positions.append(RobotAt[(t, r, c)])
                else:
                    # Must NOT be at an obstacle
                    solver.add(Not(RobotAt[(t, r, c)]))
        
        # At least one free cell is True
        solver.add(Or(possible_positions))

        # Exactly one free cell is True -> pairwise mutual exclusion
        for i in range(len(possible_positions)):
            for j in range(i+1, len(possible_positions)):
                solver.add(Implies(possible_positions[i], Not(possible_positions[j])))

    #
    # 3. Initial state constraint: RobotAt(0, start_row, start_col) is True
    #
    solver.add(RobotAt[(0, start[0], start[1])])
    # and the robot is not anywhere else at time t=0
    for r in range(rows):
        for c in range(cols):
            if (r, c) != start and grid[r][c] != '#':
                solver.add(Not(RobotAt[(0, r, c)]))

    #
    # 4. Goal constraint: Robot must be at goal cell at time max_t
    #    (Alternatively, you can allow "any time <= max_t" to reach the goal by
    #     relaxing this constraint or duplicating it at each time step.)
    #
    solver.add(RobotAt[(max_t, goal[0], goal[1])])

    #
    # 5. Action variables: Move(t, a) for a in {0,1,2,3} = {Up, Down, Left, Right}
    #
    Move = {}
    for t in range(max_t):
        for a in range(4):
            Move[(t, a)] = Bool(f"Move_{t}_{a}")

        # Exactly one action must be taken at each time step t
        solver.add(Or([Move[(t, a)] for a in range(4)]))
        for a1 in range(4):
            for a2 in range(a1+1, 4):
                solver.add(Implies(Move[(t, a1)], Not(Move[(t, a2)])))

    #
    # 6. Define a helper function to "slide" in a direction until hitting
    #    an obstacle/boundary.
    #
    def slide(r, c, dr, dc):
        nr, nc = r, c
        while True:
            rr = nr + dr
            cc = nc + dc
            # If out of bounds or hits an obstacle, stop
            if not (0 <= rr < rows and 0 <= cc < cols):
                return (nr, nc)
            if grid[rr][cc] == '#':
                return (nr, nc)
            # Otherwise, keep sliding
            nr, nc = rr, cc

    directions = [(-1, 0),  # up
                  ( 1, 0),  # down
                  ( 0,-1),  # left
                  ( 0, 1)]  # right

    #
    # 7. Transition constraints:
    #    If RobotAt(t, r, c) and Move(t, a), then RobotAt(t+1, r', c') where (r', c')
    #    is the cell you end up in after sliding from (r, c) in direction 'a'.
    #
    for t in range(max_t):
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] != '#':
                    for a, (dr, dc) in enumerate(directions):
                        # Where do we end if we move from (r,c) in this direction?
                        (r_next, c_next) = slide(r, c, dr, dc)
                        solver.add(Implies(
                            And(RobotAt[(t, r, c)], Move[(t, a)]),
                            RobotAt[(t+1, r_next, c_next)])
                        )

    return solver, RobotAt

