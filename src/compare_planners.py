# compare_planners.py
import time
from sat_encoding import encode_sat_plan
from sat_planner import plan_sat
from search_planner import plan_bfs
from visualize_sat import visualize_path_sat
from visualize_bfs import visualize_path_bfs

def load_map(filename):
    with open(filename, 'r') as f:
        return [list(line.rstrip('\n')) for line in f]

def find_positions(grid):
    start = goal = None
    obstacles = []
    boxes = []

    for r in range(len(grid)):
        for c in range(len(grid[0])):
            ch = grid[r][c]
            if ch == 'S':
                start = (r, c)
            elif ch == 'G':
                goal = (r, c)
            elif ch == '#':
                obstacles.append((r, c))
            elif ch == 'B':
                boxes.append((r, c))

    if not start:
        start = (0, 0)
    if not goal:
        goal = (0, 0)
    return start, goal, obstacles, boxes

def compare_planners(map_file, max_t=20):
    """
    Compare SAT-based planner and BFS search on the given map
    Run BFS first, then SAT, then compare results
    """
    # Load the map
    grid = load_map(map_file)
    start, goal, obstacles, boxes = find_positions(grid)
    
    print(f"Map: {map_file}")
    print(f"Grid size: {len(grid)}x{len(grid[0])}")
    print(f"Start: {start}, Goal: {goal}")
    print(f"Boxes: {boxes}")
    
    results = {
        'sat': {'solved': False, 'time': 0, 'path_length': 0},
        'bfs': {'solved': False, 'time': 0, 'path_length': 0, 'nodes': 0}
    }
    
    # Run BFS search first
    print("\n===== BFS Search =====")
    bfs_start_time = time.time()
    bfs_result = plan_bfs(grid, start, goal, boxes, obstacles, max_depth=max_t)
    bfs_time = time.time() - bfs_start_time
    
    if bfs_result:
        robot_path, box_paths, nodes_expanded = bfs_result
        
        print(f"BFS solution found")
        print(f"Path length: {len(robot_path)}")
        print(f"Time taken: {bfs_time:.8f} seconds")
        print(f"Nodes expanded: {nodes_expanded}")
        
        # Save results
        results['bfs']['solved'] = True
        results['bfs']['time'] = bfs_time
        results['bfs']['path_length'] = len(robot_path)
        results['bfs']['nodes'] = nodes_expanded
        
        # Visualize
        visualize_path_bfs(grid, robot_path, box_paths)
        print("BFS solution saved as path.png and path.gif")
        
        # Save path
        with open("bfs_path.txt", "w") as f:
            f.write(str(robot_path))
    else:
        print("BFS search failed to find a solution")
        results['bfs']['time'] = bfs_time
    
    # Now run SAT planner
    print("\n===== SAT-Based Planning =====")
    print("\nPlanner still running, it may take about 10 minutes...\n")
    sat_start_time = time.time()
    sat_horizon = -1
    
    for t in range(1, max_t):
        print(f"Trying SAT with horizon t={t}...")
        solver, RobotAt, BoxAt = encode_sat_plan(grid, start, goal, boxes, obstacles, t)
        sat_result = plan_sat(solver, RobotAt, BoxAt, t, (len(grid), len(grid[0])), len(boxes),grid)
        
        if sat_result:
            robot_path, box_paths = sat_result
            sat_time = time.time() - sat_start_time
            sat_horizon = t
            
            print(f"SAT solution found at horizon {t}")
            print(f"Path length: {len(robot_path)}")
            print(f"Time taken: {sat_time:.8f} seconds")
            
            # Save results
            results['sat']['solved'] = True
            results['sat']['time'] = sat_time
            results['sat']['path_length'] = len(robot_path)
            
            # Visualize
            visualize_path_sat(grid, robot_path, box_paths)
            print("SAT solution saved as path.png and path.gif")
            
            # Save path
            with open("sat_path.txt", "w") as f:
                f.write(str(robot_path))
            
            break
    else:
        sat_time = time.time() - sat_start_time
        print(f"SAT planner failed to find solution within {max_t} steps")
        results['sat']['time'] = sat_time
    
    # Print comparison
    print("\n===== Comparison Results =====")
    print(f"{'Method':<8} {'Solved':<8} {'Time (s)':<10} {'Path Length':<12} {'SAT Horizon/Nodes':<15}")
    print("-" * 60)
    print(f"BFS      {results['bfs']['solved']:<8} {results['bfs']['time']:<10.8f} {results['bfs']['path_length']:<12} {results['bfs']['nodes']:<15}")
    print(f"SAT      {results['sat']['solved']:<8} {results['sat']['time']:<10.8f} {results['sat']['path_length']:<12} {sat_horizon if sat_horizon >= 0 else 'N/A':<15}")
    
    # Analysis
    # Analysis
    if results['sat']['solved'] and results['bfs']['solved']:
        print("\n===== Analysis =====")
        
        # Time comparison
        if results['sat']['time'] > 0 and results['bfs']['time'] > 0:
            time_ratio = results['bfs']['time'] / results['sat']['time']
            if time_ratio > 1:
                print(f"SAT planning was {time_ratio:.8f}x faster than BFS search")
            else:
                print(f"BFS search was {1/time_ratio:.8f}x faster than SAT planning")
        else:
            if results['bfs']['time'] == 0:
                print("BFS search was extremely fast (near-zero execution time)")
            elif results['sat']['time'] == 0:
                print("SAT planning was extremely fast (near-zero execution time)")
        
        # Path quality comparison
        if results['sat']['path_length'] < results['bfs']['path_length']:
            print(f"SAT planning found a shorter path ({results['sat']['path_length']} steps vs {results['bfs']['path_length']} steps)")
        elif results['sat']['path_length'] > results['bfs']['path_length']:
            print(f"BFS search found a shorter path ({results['bfs']['path_length']} steps vs {results['sat']['path_length']} steps)")
        else:
            print(f"Both methods found paths of the same length ({results['sat']['path_length']} steps)")

    return results

if __name__ == "__main__":
    import sys
    import os
    
    # Get the path to the 'maps' directory one level up from 'src'
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(current_dir)
    maps_dir = os.path.join(parent_dir, "maps")
    
    # Set default map path and handle command line arguments
    default_map = os.path.join(maps_dir, "icepath.txt")
    
    # If user provided a map path without full path, assume it's relative to maps_dir
    map_file = sys.argv[1] if len(sys.argv) > 1 else default_map
    if len(sys.argv) > 1 and not os.path.isabs(map_file) and not map_file.startswith('../'):
        map_file = os.path.join(maps_dir, map_file)
        
    max_t = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    
    print(f"Using map file: {map_file}")
    compare_planners(map_file, max_t)