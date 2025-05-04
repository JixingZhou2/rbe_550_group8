from compare_planners import compare_planners

if __name__ == "__main__":
    import sys
    import os
    
    # Get the path to the 'maps' directory one level up from 'src'
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(current_dir)
    maps_dir = os.path.join(parent_dir, "maps")
    
    # Set default map path and handle command line arguments
    default_map = os.path.join(maps_dir, "scene1.txt")
    
    # If user provided a map path without full path, assume it's relative to maps_dir
    map_file = sys.argv[1] if len(sys.argv) > 1 else default_map
    if len(sys.argv) > 1 and not os.path.isabs(map_file) and not map_file.startswith('../'):
        map_file = os.path.join(maps_dir, map_file)
        
    max_t = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    
    print(f"Using map file: {map_file}")
    compare_planners(map_file, max_t)