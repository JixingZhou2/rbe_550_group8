from z3 import *

def encode_sat_plan(grid, start, goal, boxes, obstacles, max_t):
    """
    Encode a SAT instance for the 'Sokoban on Ice' domain.
    We have:
      - A robot on an icy floor, which slides until blocked.
      - Boxes that can be pushed by the robot; a box also slides
        until it is blocked.
      - Static obstacles (including boundaries, if desired).
      - A start and goal cell for the robot.

    The result is a Z3 solver containing constraints that guarantee:
      - The robot and boxes occupy valid cells at each time step.
      - When the robot moves in a direction, it slides to its stopping point.
      - If it encounters a box, it pushes that box along until that box stops.
      - No two boxes can overlap, and boxes cannot overlap obstacles.
      - The robot must be at the goal cell at time max_t.

    :param grid:   2D list of single-character strings (e.g., '.', '#', etc.).
                   '#' means obstacle; you can treat out-of-bounds as obstacles too.
    :param start:  (row, col) for the robot's starting position
    :param goal:   (row, col) for the robot's target position
    :param boxes:  List of (row, col) pairs for the initial box positions
    :param obstacles: Not strictly needed if '#' is placed in grid, but can be used
                      to keep track of obstacle positions separately if you want.
    :param max_t:  Maximum plan length (time horizon)

    :return: (solver, RobotAt, BoxAt)
             - solver  : a Z3 solver with all constraints
             - RobotAt : dict keyed by (t, r, c) -> Bool variable
             - BoxAt   : dict keyed by (t, r, c, b_id) -> Bool variable
    """
    solver = Solver()
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0

    # Directions: up, down, left, right
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # -------------------------------------------------
    # 1) Declare Boolean variables
    # -------------------------------------------------
    # RobotAt(t, r, c):  robot is in cell (r, c) at time t
    RobotAt = {}
    # BoxAt(t, r, c, b): box b is in cell (r, c) at time t
    BoxAt = {}
    # Move(t, a): at time t, the robot moves in direction a (0..3)
    Move = {}

    # RobotAt and BoxAt
    for t in range(max_t + 1):
        for r in range(rows):
            for c in range(cols):
                RobotAt[(t, r, c)] = Bool(f"RobotAt_{t}_{r}_{c}")
                for b_id in range(len(boxes)):
                    BoxAt[(t, r, c, b_id)] = Bool(f"BoxAt_{t}_{r}_{c}_b{b_id}")

    # Move(t, a)
    for t in range(max_t):
        for a in range(4):
            Move[(t, a)] = Bool(f"Move_{t}_{a}")

        # Exactly one move at each time step
        solver.add(Or([Move[(t, a)] for a in range(4)]))
        for a1 in range(4):
            for a2 in range(a1 + 1, 4):
                solver.add(Implies(Move[(t, a1)], Not(Move[(t, a2)])))

    # -------------------------------------------------
    # 2) Robot exclusivity constraints
    #    - At each time, the robot must occupy exactly one non-obstacle cell.
    # -------------------------------------------------
    for t in range(max_t + 1):
        possible_positions = []
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] != '#':  # not an obstacle
                    possible_positions.append(RobotAt[(t, r, c)])
                else:
                    # Cannot occupy an obstacle
                    solver.add(Not(RobotAt[(t, r, c)]))
        # Must occupy at least one cell
        solver.add(Or(possible_positions))
        # Must occupy at most one cell (pairwise exclusivity)
        for i in range(len(possible_positions)):
            for j in range(i + 1, len(possible_positions)):
                solver.add(Implies(possible_positions[i],
                                   Not(possible_positions[j])))

    # -------------------------------------------------
    # 3) Box exclusivity constraints
    #    - Each box occupies exactly one non-obstacle cell at any time.
    #    - No two boxes may overlap at any time.
    # -------------------------------------------------
    for t in range(max_t + 1):
        # For each box
        for b_id in range(len(boxes)):
            box_positions = []
            for r in range(rows):
                for c in range(cols):
                    if grid[r][c] != '#':  # not an obstacle
                        box_positions.append(BoxAt[(t, r, c, b_id)])
                    else:
                        solver.add(Not(BoxAt[(t, r, c, b_id)]))
            # Must be in at least one cell
            solver.add(Or(box_positions))
            # Must be in at most one cell
            for i in range(len(box_positions)):
                for j in range(i + 1, len(box_positions)):
                    solver.add(Implies(box_positions[i],
                                       Not(box_positions[j])))

        # No two boxes in the same cell
        for r in range(rows):
            for c in range(cols):
                # Gather all "BoxAt(t, r, c, b_id)" for b_id
                in_same_cell = [BoxAt[(t, r, c, b_id)]
                                for b_id in range(len(boxes))]
                # If one is true, the others must be false
                for i in range(len(in_same_cell)):
                    for j in range(i + 1, len(in_same_cell)):
                        solver.add(Implies(in_same_cell[i],
                                           Not(in_same_cell[j])))

    # -------------------------------------------------
    # 4) Initial state constraints
    # -------------------------------------------------
    # Robot's initial position
    solver.add(RobotAt[(0, start[0], start[1])])
    for r in range(rows):
        for c in range(cols):
            if (r, c) != start and grid[r][c] != '#':
                solver.add(Not(RobotAt[(0, r, c)]))

    # Boxes' initial positions
    for b_id, (br, bc) in enumerate(boxes):
        solver.add(BoxAt[(0, br, bc, b_id)])
        for r in range(rows):
            for c in range(cols):
                if (r, c) != (br, bc) and grid[r][c] != '#':
                    solver.add(Not(BoxAt[(0, r, c, b_id)]))

    # -------------------------------------------------
    # 5) Goal state constraint: Robot at goal at time max_t
    # -------------------------------------------------
    solver.add(RobotAt[(max_t, goal[0], goal[1])])

    # -------------------------------------------------
    # 6) Helper to simulate 'sliding' with possible pushing of a box
    #    (pure Python function to figure out final positions)
    # -------------------------------------------------
    def slide_robot_and_box(r, c, dr, dc, box_positions):
        """
        - r,c: robot start
        - dr,dc: direction of movement
        - box_positions: dict {b_id: (br, bc)} for each box b_id
                         representing where that box is at the *start* of the move.
        Returns:
          (r_robot_final, c_robot_final, new_box_positions_dict)
            * r_robot_final,c_robot_final are final robot coords
            * new_box_positions_dict is a dict of all boxes' final positions
        """
        nr, nc = r, c  # robot's position as it slides
        new_bpos = dict(box_positions)  # copy so we can modify

        while True:
            rr = nr + dr
            cc = nc + dc
            # If we go out of bounds or into a static obstacle, stop.
            if not (0 <= rr < rows and 0 <= cc < cols):
                break
            if grid[rr][cc] == '#':
                break

            # Check if there's a box there
            box_id_at_next = None
            for b_id, (bxr, bxc) in new_bpos.items():
                if bxr == rr and bxc == cc:
                    box_id_at_next = b_id
                    break

            # If no box, keep sliding the robot
            if box_id_at_next is None:
                nr, nc = rr, cc
                continue

            # Otherwise, try to push that box
            push_r, push_c = rr, cc
            while True:
                push_r2 = push_r + dr
                push_c2 = push_c + dc
                # Stop if out of bounds / obstacle
                if not (0 <= push_r2 < rows and 0 <= push_c2 < cols):
                    break
                if grid[push_r2][push_c2] == '#':
                    break
                # Stop if another box is there
                blocked = False
                for ob_id, (ob_r, ob_c) in new_bpos.items():
                    if ob_id != box_id_at_next and ob_r == push_r2 and ob_c == push_c2:
                        blocked = True
                        break
                if blocked:
                    break
                # Otherwise, keep sliding the box
                push_r, push_c = push_r2, push_c2

            # push_r, push_c is final box position
            new_bpos[box_id_at_next] = (push_r, push_c)
            # The robot stops just behind the box
            nr, nc = push_r - dr, push_c - dc
            # Pushing done, break from main loop
            break

        return nr, nc, new_bpos

    # -------------------------------------------------
    # 7) State transitions for each time step
    #    If RobotAt(t, r, c) & Move(t, a) & for each box b -> BoxAt(t, ...),
    #    we figure out the final (robot, box) positions and constrain
    #    RobotAt(t+1, r', c'), BoxAt(t+1, ...).
    # -------------------------------------------------
    for t in range(max_t):
        for r in range(rows):
            for c in range(cols):
                # If (r,c) is an obstacle, skip
                if grid[r][c] == '#':
                    continue

                for a, (dr, dc) in enumerate(directions):
                    # The condition: RobotAt(t, r, c) & Move(t, a)
                    precond = And(RobotAt[(t, r, c)], Move[(t, a)])

                    # We now need to consider all possible positions of all boxes at time t.
                    # For each 'configuration' of boxes, we figure out the final arrangement.
                    # We'll do a nested loop for each box to define constraints of the form:
                    #
                    #    If [ RobotAt(t,r,c), Move(t,a), and for each box b, BoxAt(t, br, bc, b) ],
                    #    then [ RobotAt(t+1, r_final, c_final), 
                    #            for each box b, BoxAt(t+1, br_final, bc_final, b) ].
                    #
                    # Because enumerating all box positions can be large, a more clever 
                    # "functional" encoding is often done. But here, we show the direct approach.

                    # Build up a big loop over all box placements
                    # We'll store each combination of placements in a list (or build constraints on the fly).
                    # For each combination, we do an Implies(... => ...).
                    # This is naive but demonstrates the idea.

                    # 1) Gather all (b_id, br, bc) tuples for each possible cell
                    #    We'll do a big nested set of loops.

                    def all_box_positions(b_id=0, current_assignments=None):
                        """
                        Recursively enumerate all possible placements
                        for boxes b_id..end.
                        current_assignments: {b: (row, col)} so far
                        Yields a full dictionary of b->(row, col).
                        """
                        if current_assignments is None:
                            current_assignments = {}
                        if b_id == len(boxes):
                            yield current_assignments
                            return
                        # For box b_id, iterate over all cells
                        for br in range(rows):
                            for bc in range(cols):
                                if grid[br][bc] != '#':
                                    # record it
                                    current_assignments[b_id] = (br, bc)
                                    yield from all_box_positions(b_id + 1,
                                                                 current_assignments)
                        # remove the assignment for cleanliness
                        current_assignments.pop(b_id, None)

                    # Now we generate constraints
                    for box_config in all_box_positions():
                        # box_config is a dict {b_id: (br, bc)}

                        # Build the condition that at time t, each box b_id is in the correct (br,bc)
                        box_condition = []
                        for b_id, (br, bc) in box_config.items():
                            box_condition.append(BoxAt[(t, br, bc, b_id)])

                        # The big IF part
                        condition = And(precond, *box_condition)

                        # If that is satisfied, we find final positions
                        # using slide_robot_and_box. We give it the entire box_config at time t.
                        r_final, c_final, new_box_positions = slide_robot_and_box(
                            r, c, dr, dc, box_config
                        )

                        # The THEN part: RobotAt(t+1, r_final, c_final)
                        # and for each box b, BoxAt(t+1, new_r, new_c, b).
                        effects = [RobotAt[(t + 1, r_final, c_final)]]
                        # For each box
                        for b_id2, (br2, bc2) in new_box_positions.items():
                            effects.append(BoxAt[(t + 1, br2, bc2, b_id2)])
                        # Also, for cells where the box is not supposed to be, we assert Not(BoxAt(t+1, ...)).
                        # Because each box must appear in exactly one cell, the mutual-exclusion constraints 
                        # from earlier handle that. So we primarily need to ensure it can be *somewhere*.

                        # Meanwhile, any box that new_box_positions does not assign to (which should not happen),
                        # we could ensure is not present, but we've enumerated them all, so we are fine.

                        # Build an implication: condition => (all effects are True).
                        # We'll join the effects with And(...).
                        solver.add(Implies(condition, And(effects)))

    return solver, RobotAt, BoxAt
