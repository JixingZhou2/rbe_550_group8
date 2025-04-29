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

    num_boxes = len(boxes)

    def any_box_at(t, r, c):
        if num_boxes == 0:
            return BoolVal(False) 
        """∃ b . BoxAt(t,r,c,b)"""
        return Or([BoxAt[(t, r, c, b)] for b in range(num_boxes)])


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
        # # Must occupy at least one cell
        # solver.add(Or(possible_positions))
        # # Must occupy at most one cell (pairwise exclusivity)
        # for i in range(len(possible_positions)):
        #     for j in range(i + 1, len(possible_positions)):
        #         solver.add(Implies(possible_positions[i],
        #                            Not(possible_positions[j]))) # too slow

        solver.add(PbEq([(v, 1) for v in possible_positions], 1))

    # -------------------------------------------------
    # 3) Box exclusivity constraints
    #    - Each box occupies exactly one non-obstacle cell at any time.
    #    - No two boxes may overlap at any time.
    # -------------------------------------------------
    if num_boxes > 0:
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
                # # Must be in at least one cell
                # solver.add(Or(box_positions))
                # # Must be in at most one cell
                # for i in range(len(box_positions)):
                #     for j in range(i + 1, len(box_positions)):
                #         solver.add(Implies(box_positions[i],
                #                            Not(box_positions[j]))) # too slow
    
                solver.add(PbEq([(v, 1) for v in box_positions], 1))
    
            # No two boxes in the same cell
            for r in range(rows):
                for c in range(cols):
                    # Gather all "BoxAt(t, r, c, b_id)" for b_id
                    in_same_cell = [BoxAt[(t, r, c, b_id)]
                                    for b_id in range(len(boxes))]
                    # # If one is true, the others must be false
                    # for i in range(len(in_same_cell)):
                    #     for j in range(i + 1, len(in_same_cell)):
                    #         solver.add(Implies(in_same_cell[i],
                    #                            Not(in_same_cell[j]))) # too slow
    
                    solver.add(AtMost(*in_same_cell, 1))

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
    def static_slide(r0, c0, dr, dc):
        r, c = r0, c0
        while True:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols): break
            if grid[nr][nc] == '#':                     break
            r, c = nr, nc
        return r, c

    STATIC_STOP = {(r, c, a): static_slide(r, c, *d)
                   for r in range(rows) for c in range(cols) if grid[r][c] != '#'
                   for a, d in enumerate(directions)}

    for t in range(max_t):
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] == '#':            
                    continue
                for a, (dr, dc) in enumerate(directions):
                    move_here = And(RobotAt[(t, r, c)], Move[(t, a)])

                    
                    cells = []
                    rr, cc = r, c
                    while True:
                        rr, cc = rr + dr, cc + dc
                        if not (0 <= rr < rows and 0 <= cc < cols): break
                        if grid[rr][cc] == '#':                   break
                        cells.append((rr, cc))
                    if not cells:
                        solver.add(Not(And(RobotAt[(t, r, c)], Move[(t, a)])))
                        continue        

                    # -------- nobox ----------
                    no_box_cond = And([Not(any_box_at(t, x, y)) for (x, y) in cells])
                    dst_r, dst_c = STATIC_STOP[(r, c, a)]
                    for b_id in range(num_boxes):
                        for x in range(rows):
                            for y in range(cols):
                                stay_same = Implies(
                                    And(move_here, no_box_cond, BoxAt[(t, x, y, b_id)]),
                                    BoxAt[(t + 1, x, y, b_id)]
                                )
                                solver.add(stay_same)
                    solver.add(Implies(And(move_here, no_box_cond),
                                       RobotAt[(t + 1, dst_r, dst_c)]))

                    # ------------------
                    push_alternatives = []     #  completeness
                    clear_prefix = True        # z3

                    for (bx, by) in cells:
                        # path_clear   = And(clear_prefix,
                        #                    Not(any_box_at(t, bx, by)))
                        # box_here_any = any_box_at(t, bx, by)
                        # clear_prefix = And(clear_prefix, Not(box_here_any))

                        # 
                        # push_precond = And(move_here, clear_prefix, box_here_any)
                        # if push_precond is False:          
                        # continue
                        prev_clear   = clear_prefix                 # 
                        box_here_any = any_box_at(t, bx, by)
                        clear_prefix = And(prev_clear, Not(box_here_any))   # 
                        # 
                        if box_here_any is False:
                            continue

                        # 
                        for b in range(num_boxes):
                            #cond = And(move_here, clear_prefix, BoxAt[(t, bx, by, b)])
                            cond = And(move_here, prev_clear,BoxAt[(t, bx, by, b)])
                            # 
                            stop_x, stop_y = bx, by
                            slide_blocks = []
                            slide_clear = BoolVal(True)
                            while True:
                                nx, ny = stop_x + dr, stop_y + dc
                                if not (0 <= nx < rows and 0 <= ny < cols): break
                                if grid[nx][ny] == '#':                   break
                                # 
                                slide_clear = And(slide_clear, Not(any_box_at(t, nx, ny)))
                                stop_x, stop_y = nx, ny

                            # 
                            # path_ok = And(*slide_blocks)

                            # robot_next = (bx - dr, by - dc)
                            # effect = And(
                            #     path_ok,
                            #     RobotAt[(t + 1, robot_next[0], robot_next[1])],
                            #     BoxAt[(t + 1, stop_x, stop_y, b)]
                            # )
                            # solver.add(Implies(cond, effect))
                            #
                            cond = And(cond, slide_clear)
                            robot_next = (bx - dr, by - dc)   # 机器人停在箱子后一格
                            effect = And(RobotAt[(t + 1, robot_next[0], robot_next[1])],
                                         BoxAt[(t + 1, stop_x, stop_y, b)])
                            solver.add(Implies(cond, effect))  



                            push_alternatives.append(cond)

                            # 
                            for b2 in range(num_boxes):
                                if b2 == b:           # 
                                    continue
                                for x in range(rows):
                                    for y in range(cols):
                                        stay = Implies(cond & BoxAt[(t, x, y, b2)],
                                                       BoxAt[(t + 1, x, y, b2)])
                                        solver.add(stay)

                    # --------  completeness ----------
                    valid_outcome = Or(no_box_cond, *push_alternatives)
                    solver.add(Implies(move_here, valid_outcome))


    with open("sokoban_on_ice.smt2", "w") as f:
        f.write(solver.to_smt2())

    return solver, RobotAt, BoxAt
