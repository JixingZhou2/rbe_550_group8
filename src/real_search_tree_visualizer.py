# real_search_tree_visualizer.py
import networkx as nx
import matplotlib.pyplot as plt
import os

def visualize_real_bfs_tree(all_states, goal_state, grid):
   
    print(f"Building visualization for {len(all_states)} actual BFS states...")
    
    # Create a directed graph
    G = nx.DiGraph()
    
    # Add all nodes to the graph
    for state_hash, state in all_states.items():
        # Create a unique node ID based on the hash
        node_id = f"node_{state_hash}"
        
        # Determine if this state is in the solution path
        in_solution = False
        
        # Add node with attributes
        G.add_node(node_id, 
                  pos=state.robot_pos, 
                  boxes=state.boxes, 
                  depth=state.depth,
                  in_solution=in_solution)
    
    # Add edges based on parent relationships
    for state_hash, state in all_states.items():
        if state.parent:
            parent_hash = hash(state.parent)
            if parent_hash in all_states:  # Make sure the parent is in our set
                G.add_edge(f"node_{parent_hash}", f"node_{state_hash}")
    
    # Now mark the solution path
    # Trace back from goal to start
    solution_path = []
    current = goal_state
    while current:
        solution_path.append(current)
        current = current.parent
    solution_path.reverse()
    
    # Mark solution path nodes
    for state in solution_path:
        state_hash = hash(state)
        node_id = f"node_{state_hash}"
        if node_id in G.nodes:
            G.nodes[node_id]['in_solution'] = True
    
    print(f"Created graph with {len(G.nodes())} nodes and {len(G.edges())} edges")
    
    # Increase figure size for more nodes
    plt.figure(figsize=(30, 20))
    
    # Use a better layout for tree spreading - try different layouts
    try:
        # Try force-directed layout with strong repulsion for node separation
        pos = nx.spring_layout(G, k=1.5, iterations=200, seed=42)
    except:
        try:
            # Try kamada-kawai layout which often gives good spacing
            pos = nx.kamada_kawai_layout(G)
        except:
            # Fall back to spring layout with stronger repulsion
            pos = nx.spring_layout(G, k=2.0, iterations=300)
    
    # Draw solution path nodes larger and in a different color
    solution_nodes = [n for n, attrs in G.nodes(data=True) if attrs.get('in_solution', False)]
    other_nodes = [n for n, attrs in G.nodes(data=True) if not attrs.get('in_solution', False)]
    
    nx.draw_networkx_nodes(G, pos, nodelist=solution_nodes, node_color='red', node_size=300)
    nx.draw_networkx_nodes(G, pos, nodelist=other_nodes, node_color='blue', node_size=80)
    
    # Draw edges, solution path edges thicker
    solution_edges = [(u, v) for u, v in G.edges() if u in solution_nodes and v in solution_nodes]
    other_edges = [(u, v) for u, v in G.edges() if not (u in solution_nodes and v in solution_nodes)]
    
    nx.draw_networkx_edges(G, pos, edgelist=solution_edges, width=3, edge_color='red')
    nx.draw_networkx_edges(G, pos, edgelist=other_edges, width=0.8, alpha=0.5)
    
    # Only label solution path nodes to reduce clutter
    solution_labels = {n: f"R:{G.nodes[n]['pos']}" for n in solution_nodes}
    nx.draw_networkx_labels(G, pos, labels=solution_labels, font_size=10, font_weight='bold')
    
    # Add color legend
    plt.plot([], [], 'ro', markersize=10, label='Solution Path')
    plt.plot([], [], 'bo', markersize=5, label='Explored States')
    plt.legend(fontsize=14)
    
    plt.title(f"Actual BFS Search Tree (Nodes Explored: {len(all_states)})", fontsize=16)
    plt.axis('off')
    plt.tight_layout()
    
    # Save high-resolution image
    output_dir = "BFS_search_tree_visualization"
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(os.path.join(output_dir, "actual_bfs_tree.png"), dpi=150)
    plt.close()
    
    print(f"Actual BFS tree visualization saved to {os.path.join(output_dir, 'actual_bfs_tree.png')}")
    
    # Also create a simplified version with essential nodes only
    create_simplified_actual_tree(solution_path, G, all_states)

def create_simplified_actual_tree(solution_path, full_graph, all_states):
    """
    Create a simplified version of the actual search tree showing essential structure.
    """
    G_simple = nx.DiGraph()
    
    # Add solution path nodes
    for i, state in enumerate(solution_path):
        state_hash = hash(state)
        node_id = f"sol_{state_hash}"
        G_simple.add_node(node_id, pos=state.robot_pos, depth=state.depth, in_solution=True)
        
        # Add edge to previous node in solution path
        if i > 0:
            prev_hash = hash(solution_path[i-1])
            G_simple.add_edge(f"sol_{prev_hash}", node_id)
    
    # Add some representative branch points
    # Find branch points by looking at nodes with multiple children
    branch_points = []
    for node in full_graph.nodes():
        if full_graph.out_degree(node) > 1:
            branch_points.append(node)
    
    # Add a subset of branch points to show the search structure
    added_branches = 0
    max_branches = min(50, len(branch_points))  # Limit branches but use all if fewer than 50
    
    for bp in branch_points:
        if added_branches >= max_branches:
            break
            
        # Skip if this is a solution node (already added)
        if full_graph.nodes[bp].get('in_solution', False):
            continue
            
        # Get original state hash from node id (node_HASH)
        orig_hash = int(bp.split('_')[1])
        if orig_hash not in all_states:
            continue
            
        state = all_states[orig_hash]
        branch_id = f"branch_{orig_hash}"
        
        # Add the branch point
        G_simple.add_node(branch_id, pos=state.robot_pos, depth=state.depth, in_solution=False)
        
        # Add edge from parent if possible
        if state.parent:
            parent_hash = hash(state.parent)
            # Check if parent is a solution node
            parent_in_solution = False
            for sol_state in solution_path:
                if hash(sol_state) == parent_hash:
                    parent_in_solution = True
                    break
                    
            if parent_in_solution:
                G_simple.add_edge(f"sol_{parent_hash}", branch_id)
            else:
                # Add non-solution parent if not already added
                if f"branch_{parent_hash}" in G_simple.nodes:
                    G_simple.add_edge(f"branch_{parent_hash}", branch_id)
                    
        # Add some children of this branch point
        children = list(full_graph.successors(bp))
        for i, child in enumerate(children[:3]):  # Limit to 3 children per branch
            # Get original hash
            try:
                child_hash = int(child.split('_')[1])
                if child_hash in all_states:
                    child_state = all_states[child_hash]
                    child_id = f"child_{child_hash}_{i}"
                    G_simple.add_node(child_id, pos=child_state.robot_pos, depth=child_state.depth, in_solution=False)
                    G_simple.add_edge(branch_id, child_id)
            except:
                continue
                
        added_branches += 1
    
    # Draw the simplified graph
    plt.figure(figsize=(25, 18))
    
    # Use a clearer layout for the simplified tree
    try:
        pos = nx.kamada_kawai_layout(G_simple)
    except:
        pos = nx.spring_layout(G_simple, k=1.0, iterations=100)
    
    # Draw solution path nodes
    solution_nodes = [n for n, attrs in G_simple.nodes(data=True) if attrs.get('in_solution', True)]
    other_nodes = [n for n, attrs in G_simple.nodes(data=True) if not attrs.get('in_solution', True)]
    
    nx.draw_networkx_nodes(G_simple, pos, nodelist=solution_nodes, node_color='red', node_size=300)
    nx.draw_networkx_nodes(G_simple, pos, nodelist=other_nodes, node_color='blue', node_size=150)
    
    # Draw edges
    solution_edges = [(u, v) for u, v in G_simple.edges() if u in solution_nodes and v in solution_nodes]
    other_edges = [(u, v) for u, v in G_simple.edges() if not (u in solution_nodes and v in solution_nodes)]
    
    nx.draw_networkx_edges(G_simple, pos, edgelist=solution_edges, width=3, edge_color='red')
    nx.draw_networkx_edges(G_simple, pos, edgelist=other_edges, width=1, alpha=0.7)
    
    # Add simple labels
    sol_labels = {n: f"S{i}" for i, n in enumerate(solution_nodes)}
    nx.draw_networkx_labels(G_simple, pos, labels=sol_labels, font_size=10, font_weight='bold')
    
    # Add color legend
    plt.plot([], [], 'ro', markersize=10, label='Solution Path')
    plt.plot([], [], 'bo', markersize=5, label='Explored States')
    plt.legend(fontsize=14)
    
    plt.title(f"Simplified Actual BFS Tree (Total Nodes Explored: {len(all_states)})", fontsize=16)
    plt.axis('off')
    plt.tight_layout()
    
    # Save the simplified tree
    output_dir = "visualization"
    plt.savefig(os.path.join(output_dir, "simplified_actual_bfs_tree.png"), dpi=200)
    plt.close()
    
    print(f"Simplified actual BFS tree visualization saved to {os.path.join(output_dir, 'simplified_actual_bfs_tree.png')}")