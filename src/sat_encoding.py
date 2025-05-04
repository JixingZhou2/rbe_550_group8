from z3 import *

def encode_sat_plan(grid, start, goal, boxes, obstacles, max_t):

    set_param("parallel.enable", True)          #
    set_param("smt.threads", 8)
    rows, cols  = len(grid), len(grid[0])
    num_boxes   = len(boxes)
    directions  = [(-1, 0), (1, 0), (0, -1), (0, 1)]      # UP, DOWN, LEFT, RIGHT

    # -------- pre‑compute straight paths ignoring --------
    free_cells = [(r, c) for r in range(rows) for c in range(cols)
                  if grid[r][c] != '#']

    path_cells, static_stop = {}, {}
    for r, c in free_cells:
        for a, (dr, dc) in enumerate(directions):
            seq = []
            rr, cc = r, c
            while True:
                rr += dr; cc += dc
                if not (0 <= rr < rows and 0 <= cc < cols): break
                if grid[rr][cc] == '#': break
                seq.append((rr, cc))
            path_cells[(r, c, a)]  = seq                       # robot path
            static_stop[(r, c, a)] = seq[-1] if seq else (r, c)  # box stop

    solver = Solver()
    RobotAt, BoxAt, Move = {}, {}, {}

    for t in range(max_t + 1):
        for r, c in free_cells:
            RobotAt[(t, r, c)] = Bool(f"R_{t}_{r}_{c}")
            for b in range(num_boxes):
                BoxAt[(t, r, c, b)] = Bool(f"B_{t}_{r}_{c}_{b}")

    for t in range(max_t):
        for a in range(4):
            Move[(t, a)] = Bool(f"M_{t}_{a}")
        solver.add(PbEq([(Move[(t, a)], 1) for a in range(4)], 1))   # 

    any_box_cache = {}
    def any_box(t, r, c):
        key = (t, r, c)
        if key not in any_box_cache:
            any_box_cache[key] = Or([BoxAt[(t, r, c, b)] for b in range(num_boxes)]) if num_boxes else BoolVal(False)
        return any_box_cache[key]

    # -------- unique --------
    for t in range(max_t + 1):
        solver.add(PbEq([(RobotAt[(t, r, c)], 1) for r, c in free_cells], 1))          # robot unique
        for b in range(num_boxes):
            solver.add(PbEq([(BoxAt[(t, r, c, b)], 1) for r, c in free_cells], 1))     # box unique
        for r, c in free_cells:                                                        # same cell
            solver.add(AtMost(*[BoxAt[(t, r, c, b)] for b in range(num_boxes)], 1))

    solver.add(RobotAt[(0, *start)])
    for r, c in free_cells:
        if (r, c) != start:
            solver.add(Not(RobotAt[(0, r, c)]))

    for b, (br, bc) in enumerate(boxes):
        solver.add(BoxAt[(0, br, bc, b)])
        for r, c in free_cells:
            if (r, c) != (br, bc):
                solver.add(Not(BoxAt[(0, r, c, b)]))

    solver.add(RobotAt[(max_t, *goal)])

    pushed = {(t, b): [] for t in range(max_t) for b in range(num_boxes)}

    for t in range(max_t):

        # only one direction can be moved
        for a in range(4):
            origins = [RobotAt[(t, r, c)] for r, c in free_cells if path_cells[(r, c, a)]]
            solver.add(Implies(Move[(t, a)], Or(origins)))

        for r, c in free_cells:
            for a, (dr, dc) in enumerate(directions):
                seq = path_cells[(r, c, a)]
                if not seq:
                    solver.add(Not(And(RobotAt[(t, r, c)], Move[(t, a)])))
                    continue

                move_here = And(RobotAt[(t, r, c)], Move[(t, a)])

                # 1. no box in the path
                clear_seq = And([Not(any_box(t, x, y)) for x, y in seq])
                dst_r, dst_c = static_stop[(r, c, a)]
                solver.add(Implies(And(move_here, clear_seq),
                                   RobotAt[(t + 1, dst_r, dst_c)]))
                for b in range(num_boxes):
                    for x, y in free_cells:
                        solver.add(Implies(And(move_here, clear_seq, BoxAt[(t, x, y, b)]),
                                           BoxAt[(t + 1, x, y, b)]))

                # 2. box in the path → push the first box stopped by a wall or another box
                prefix_clear_to_box = BoolVal(True)     
                for bx, by in seq:
                    box_here      = any_box(t, bx, by)
                    first_box_ok  = And(move_here, prefix_clear_to_box, box_here)

                    prefix_clear_to_box = And(prefix_clear_to_box, Not(box_here))

                    if not is_bool(first_box_ok):
                        continue

                    
                    clear_between = BoolVal(True)       
                    cx, cy = bx, by
                    while True:
                        nx, ny = cx + dr, cy + dc

                        if not (0 <= nx < rows and 0 <= ny < cols) or grid[nx][ny] == '#':
                            stop_x, stop_y = cx, cy
                            push_any = And(first_box_ok, clear_between)   
                            for b in range(num_boxes):
                                push_b = And(push_any, BoxAt[(t, bx, by, b)])
                                if str(push_b) == "False": 
                                    continue
                                robot_next = (stop_x - dr, stop_y - dc)
                                solver.add(Implies(push_b,
                                                   And(RobotAt[(t + 1, *robot_next)],
                                                       BoxAt[(t + 1, stop_x, stop_y, b)])))
                                pushed[(t, b)].append(push_b)
                            break    

                        blocker_is_box = any_box(t, nx, ny)

                        stop_x, stop_y = cx, cy
                        push_any = And(first_box_ok, clear_between, blocker_is_box)
                        for b in range(num_boxes):
                            push_b = And(push_any, BoxAt[(t, bx, by, b)])
                            solver.add(Implies(push_b,
                                               And(RobotAt[(t + 1, stop_x - dr, stop_y - dc)],
                                                   BoxAt[(t + 1, stop_x, stop_y, b)])))
                            pushed[(t, b)].append(push_b)

                        clear_between = And(clear_between, Not(blocker_is_box))
                        cx, cy = nx, ny

                push_disj = Or(*[p for b in range(num_boxes) for p in pushed[(t, b)]])
                solver.add(Implies(move_here, Or(clear_seq, push_disj)))

    for t in range(max_t):
        for b in range(num_boxes):
            pushed_b = Or(pushed[(t, b)]) if pushed[(t, b)] else BoolVal(False)
            for r, c in free_cells:
                solver.add(Implies(And(BoxAt[(t, r, c, b)], Not(pushed_b)),
                                   BoxAt[(t + 1, r, c, b)]))

    return solver, RobotAt, BoxAt
