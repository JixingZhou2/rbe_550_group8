# Project: rbe_550_group8

```
project/
├── maps/
│   ├── scene1.txt
│   └── scene2.txt
├── BFS_search_tree_visualization/
└── src/
    ├── sat_encoding.py  
    ├── sat_planner.py     
    ├── visualize_sat.py 
    ├── visualize_bfs.py   
    ├── search_tree_visualizer.py   
    ├── search_planner.py     
    ├── compare_planners.py     
    └── main.py     
```

## Installation

Before running the code, make sure to install the required Python dependency:

```bash
pip install z3-solver
pip install networkx
````

## Running the Code

To run the planner and visualize both SAT-based and BFS-based results:

```bash
python src/main.py
```

This will execute:

* **SAT-plan using Z3 SMT solver**
* **BFS-based search planner**

**Z3-sat planner may takes more than 10 minutes in scene2 map if the host CPU performance is poor**

After execution, two visualization outputs will be saved:

* `path_sat.gif`: Path found by the SAT-based planner
* `path_bfs.gif`: Path found by the BFS-based planner

## Map Files

Map files are located in the `maps/` folder. You can change the map by editing the following line in `main.py`:

```python
default_map = os.path.join(maps_dir, "scene1.txt")
```

## Map Format

Each map is a text file using the following characters:

```
#  = obstacle (wall)
S  = robot start position
G  = goal position
B  = movable box
.  = free space
```

### Example:

```
######### 
#S......# 
##..#...# 
#..G.#..# 
#.......# 
#...B...# 
#..#....# 
######### 
```

