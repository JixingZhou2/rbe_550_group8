# solution_state_builder.py
from search_planner import State

def build_solution_states(robot_path, box_paths, grid):
   
    if not robot_path:
        return None, None
    
    # Create state chain
    prev_state = None
    all_states = []
    
    for i in range(len(robot_path)):
        # Get robot and box positions at this step
        robot_pos = robot_path[i]
        boxes = []
        for box_path in box_paths:
            if i < len(box_path):
                boxes.append(box_path[i])
        
        # Create state
        state = State(robot_pos, boxes, prev_state)
        all_states.append(state)
        prev_state = state
    
    if all_states:
        return all_states[0], all_states[-1]
    return None, None

