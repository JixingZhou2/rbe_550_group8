# search_tree_visualizer.py
import networkx as nx
import matplotlib.pyplot as plt
from collections import deque
import os

def create_search_tree(start_state, goal_state):
    """
    Create a search tree visualization starting from the goal state
    and tracing back to the start state, then forward through the solution path.
    
    Args:
        start_state: The initial State object
        goal_state: The final State object that reached the goal
    """
    # Create a directed graph
    G = nx.DiGraph()
    
    # Trace back from goal to start to get the solution path
    solution_path = []
    current = goal_state
    while current:
        solution_path.append(current)
        current = current.parent
    solution_path.reverse()
    
    # Add all nodes and edges in the solution path
    for i in range(len(solution_path)):
        state = solution_path[i]
        # Create a unique node ID
        node_id = f"node_{i}"
        
        # Add node with attributes
        G.add_node(node_id, 
                  pos=state.robot_pos, 
                  boxes=state.boxes, 
                  depth=state.depth,
                  in_solution=True)
        
        # Add edge to parent
        if i > 0:
            G.add_edge(f"node_{i-1}", node_id)
    
    # Now do a BFS from the start state to add explored nodes that weren't in the solution
    queue = deque([(start_state, "node_0")])
    visited = set([str(start_state.robot_pos) + str(start_state.boxes)])
    node_counter = len(solution_path)
    
    while queue:
        state, parent_id = queue.popleft()
        
        # Skip if this state is already in our solution path
        state_key = str(state.robot_pos) + str(state.boxes)
        if state_key in visited:
            continue
        
        visited.add(state_key)
        node_id = f"node_{node_counter}"
        node_counter += 1
        
        # Add the node
        G.add_node(node_id, 
                  pos=state.robot_pos, 
                  boxes=state.boxes, 
                  depth=state.depth,
                  in_solution=False)
        
        # Add edge from parent
        G.add_edge(parent_id, node_id)
        
        # Add children to queue
        # Note: This assumes you can access child states, which might not be directly available
        # You might need to modify your State class to track children or simulate moves
        for child in get_children(state):
            child_key = str(child.robot_pos) + str(child.boxes)
            if child_key not in visited:
                queue.append((child, node_id))
    
    # Draw the graph
    plt.figure(figsize=(15, 10))
    
    # Create a hierarchical layout
    pos = nx.nx_agraph.graphviz_layout(G, prog='dot')
    
    # Draw solution path nodes larger and in a different color
    solution_nodes = [n for n, attrs in G.nodes(data=True) if attrs.get('in_solution', False)]
    other_nodes = [n for n, attrs in G.nodes(data=True) if not attrs.get('in_solution', False)]
    
    nx.draw_networkx_nodes(G, pos, nodelist=solution_nodes, node_color='red', node_size=300)
    nx.draw_networkx_nodes(G, pos, nodelist=other_nodes, node_color='blue', node_size=100)
    
    # Draw edges, solution path edges thicker
    solution_edges = [(u, v) for u, v in G.edges() if u in solution_nodes and v in solution_nodes]
    other_edges = [(u, v) for u, v in G.edges() if not (u in solution_nodes and v in solution_nodes)]
    
    nx.draw_networkx_edges(G, pos, edgelist=solution_edges, width=2, edge_color='red')
    nx.draw_networkx_edges(G, pos, edgelist=other_edges, width=1, alpha=0.5)
    
    # Add labels
    labels = {n: f"D:{attrs['depth']}\nR:{attrs['pos']}" for n, attrs in G.nodes(data=True)}
    nx.draw_networkx_labels(G, pos, labels=labels, font_size=8)
    
    plt.title(f"BFS Search Tree (Solution Length: {len(solution_path)})")
    plt.axis('off')
    plt.tight_layout()
    
    # Save to file
    output_dir = "visualization"
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(os.path.join(output_dir, "search_tree.png"), dpi=300)
    plt.close()
    
    print(f"Search tree visualization saved to {os.path.join(output_dir, 'search_tree.png')}")
    print(f"Total nodes in visualization: {len(G.nodes())}")
    print(f"Solution path length: {len(solution_path)}")

def get_children(state):
    """
    Simulate available moves from this state to get child states.
    Note: You'll need to implement this based on your specific state representation
    and movement logic.
    """
    # This is a placeholder - you'll need to implement this based on your code
    # It should return a list of valid child State objects
    return []

# Usage example
# In your main script, add:
# if bfs_result:
#     robot_path, box_paths, nodes_expanded = bfs_result
#     from search_tree_visualizer import create_search_tree
#     create_search_tree(initial_state, final_state)