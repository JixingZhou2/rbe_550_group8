# search_planner.py
from collections import deque
import time
from sat_encoding import encode_sat_plan  # Import the SAT encoding to reuse logic

class State:
    """
    Represents a state in the Sokoban on Ice problem.
    """
    def __init__(self, robot_pos, box_positions, parent=None, action=None):
        self.robot_pos = robot_pos  # (row, col)
        # Store boxes as a tuple for hashability
        self.boxes = tuple(sorted(box_positions))
        self.parent = parent  # Parent state for path reconstruction
        self.action = action  # Action that led to this state
        self.depth = 0 if parent is None else parent.depth + 1
    
    def __hash__(self):
        return hash((self.robot_pos, self.boxes))
    
    def __eq__(self, other):
        return self.robot_pos == other.robot_pos and self.boxes == other.boxes


def plan_bfs(grid, start, goal, boxes, obstacles, max_depth=50):
    """
    Solve Sokoban on Ice using BFS search
    
    Returns:
        (robot_path, box_paths, nodes_expanded) if solution found, else None
    """
    print(f"Starting BFS search from {start} to {goal} with {len(boxes)} boxes")
    start_time = time.time()
    
    # Grid dimensions
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    
    # Directions: up, down, left, right (same as in SAT encoding)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    # Create initial state
    initial_state = State(start, boxes)
    
    # BFS queue
    queue = deque([initial_state])
    
    # Visited states set
    visited = set([hash(initial_state)])
    
    # Count expanded nodes
    nodes_expanded = 0
    
    # Direct implementation of slide_robot_and_box function
    def slide_robot_and_box(r, c, dr, dc, box_positions):
        """Simulate robot sliding with possible box pushing"""
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
        
        return nr, nc, list(new_bpos.values())
    
    # BFS main loop
    while queue:
        # Get next state from queue
        state = queue.popleft()
        nodes_expanded += 1
        
        # Print progress more frequently for debugging
        if nodes_expanded % 1000 == 0:
            elapsed = time.time() - start_time
            print(f"Explored {nodes_expanded} nodes in {elapsed:.2f}s. Queue size: {len(queue)}")
        
        # Check if we've reached the goal
        if state.robot_pos == goal:
            elapsed = time.time() - start_time
            print(f"Solution found at depth {state.depth} after exploring {nodes_expanded} nodes")
            print(f"Time taken: {elapsed:.8f} seconds")
            
            # Reconstruct path
            robot_path, box_paths = reconstruct_path(state)
            return robot_path, box_paths, nodes_expanded
        
        # Check if we've reached the maximum depth
        if state.depth >= max_depth:
            continue
        
        # Try each direction
        for a, (dr, dc) in enumerate(directions):
            # Convert tuple to dict for slide_robot_and_box compatibility
            box_positions = dict(enumerate(state.boxes))
            
            # Simulate the move
            try:
                nr, nc, new_boxes = slide_robot_and_box(
                    state.robot_pos[0], state.robot_pos[1], 
                    dr, dc, 
                    box_positions
                )
                
                # Create new state
                new_state = State((nr, nc), new_boxes, state, a)
                
                # Check if we've seen this state before
                new_hash = hash(new_state)
                if new_hash not in visited:
                    visited.add(new_hash)
                    queue.append(new_state)
            except Exception as e:
                print(f"Error simulating move: {e}")
                continue
    
    # No solution found
    elapsed = time.time() - start_time
    print(f"No solution found after exploring {nodes_expanded} nodes")
    print(f"Time taken: {elapsed:.2f} seconds")
    return None

def reconstruct_path(final_state):
    """Reconstruct the path from initial state to final state"""
    # Collect states from goal to start
    states = []
    current = final_state
    while current:
        states.append(current)
        current = current.parent
    
    # Reverse to get start to goal
    states.reverse()
    
    # Extract robot path
    robot_path = [state.robot_pos for state in states]
    
    # Extract box paths
    if not states:
        return [], []
    
    num_boxes = len(states[0].boxes)
    box_paths = [[] for _ in range(num_boxes)]
    
    for state in states:
        for b_id, pos in enumerate(state.boxes):
            box_paths[b_id].append(pos)
    
    return robot_path, box_paths