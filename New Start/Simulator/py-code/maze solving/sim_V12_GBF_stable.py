import networkx as nx
import matplotlib.pyplot as plt
import math
from enum import Enum 

PRINT_SPEED = 0.3

# --- 1. Define Maze Structure (User-corrected Map) ---
# (Unchanged)
nodes_positions = {
    0:  (0, 0),    #start
    1:  (0, 20),
    2:  (0, 40),   
    3:  (0, 60),   
    4:  (0, 100), 
    5:  (0, 120),
    6:  (20, 40),   
    7:  (20, 100),   
    8:  (20, 120),
    9:  (40, 40),   
    10: (40, 60),   
    11: (40, 80),
    12: (60,40) ,
    13: (60, 80),
    14: (-20, 40), 
    15: (-20, 60), 
    16: (-20, 80),
    17: (-40, 40), 
    18: (-40, 60), 
    19: (-40, 80), 
    20: (-40, 100),
    21: (-60, 20), 
    22: (-60, 80), # Target Node
    23: (-60, 120), 
    24: (-80, 40), 
    25: (-80, 60), 
    26: (-80, 80), 
}
edges_list = [
    (0, 1), (1, 2), (1, 21), (2, 3), (2, 6), (3, 4),
    (3, 10), (3, 15), (4, 5), (4, 7), (4, 20), (7, 8),
    (9, 10), (9, 12), (10, 11), (11, 13), (12, 13),
    (14, 15), (14, 17), (15, 16), (16, 19), (17, 18),
    (17, 24), (18, 25), (19, 20), (19, 22), (22, 23),
    (22, 26), (24, 25), (25, 26),
]
START_NODE = 0
TARGET_NODE = 22

# --- 2. Create Graph ---
# (Unchanged)
G = nx.Graph()
G.add_nodes_from(nodes_positions.keys())
G.add_edges_from(edges_list)
for node, pos in nodes_positions.items():
    G.nodes[node]['pos'] = pos


# --- 3. Visualization Function ---
# (Unchanged from previous correct version)
def draw_maze(graph, robot_node=None, visited_edges=None, 
              visited_nodes=None, optimal_path=None, ax=None):
    if visited_edges is None:
        visited_edges = set()
    if visited_nodes is None:
        visited_nodes = set()
    
    pos = nx.get_node_attributes(graph, 'pos')
    
    if ax:
        ax.clear()
    else:
        fig, ax = plt.subplots(figsize=(12, 12))

    # 1. Draw all edges faintly
    nx.draw_networkx_edges(
        graph, pos, ax=ax,
        edgelist=graph.edges(),
        edge_color='grey', alpha=0.3, width=2
    )
    
    # 2. Draw visited edges strongly
    nx.draw_networkx_edges(
        graph, pos, ax=ax,
        edgelist=visited_edges,
        edge_color='blue', width=3
    )

    # 3. Draw unexplored stubs
    STUB_LENGTH = 10.0
    for u in visited_nodes:
        (x1, y1) = graph.nodes[u]['pos']
        for v in graph.neighbors(u):
            edge = tuple(sorted((u, v)))
            if edge not in visited_edges:
                (x2, y2) = graph.nodes[v]['pos']
                dx = x2 - x1
                dy = y2 - y1
                mag = math.sqrt(dx**2 + dy**2)
                if mag > 0:
                    ux = dx / mag
                    uy = dy / mag
                    x_end = x1 + STUB_LENGTH * ux
                    y_end = y1 + STUB_LENGTH * uy
                    ax.plot(
                        [x1, x_end], [y1, y_end],
                        color='orange', linewidth=3, linestyle='--'
                    )
    
    # 4. Draw the Optimal Path (MODIFIED)
    if optimal_path:
        optimal_edges = list(zip(optimal_path[:-1], optimal_path[1:]))
        
        nx.draw_networkx_edges(
            graph,
            pos,
            ax=ax,
            edgelist=optimal_edges,
            edge_color='green',  
            width=20,
            alpha=0.7        
        )

    # 5. Draw all nodes
    nx.draw_networkx_nodes(
        graph, pos, ax=ax,
        node_color='lightblue', node_size=400
    )
    
    # 6. Draw Start, Target, and Robot nodes
    nx.draw_networkx_nodes(
        graph, pos, ax=ax, nodelist=[START_NODE], 
        node_color='green', node_size=600, label='Start'
    )
    nx.draw_networkx_nodes(
        graph, pos, ax=ax, nodelist=[TARGET_NODE], 
        node_color='black', node_size=600, label='Target'
    )
    if robot_node is not None:
        nx.draw_networkx_nodes(
            graph, pos, ax=ax, nodelist=[robot_node], 
            node_color='red', node_size=600, label='Robot'
        )

    nx.draw_networkx_labels(graph, pos, ax=ax, font_color='black', font_size=9)
    ax.set_title("Maze Simulator - Nearest Frontier Exploration + Optimal Path")
    ax.legend()
    ax.axis('equal')


# --- 4. Robot Class (COMPLETELY NEW LOGIC) ---

class Robot:
    def __init__(self, graph, start_node, target_node):
        self.graph = graph
        self.start_node = start_node
        self.target_node = target_node
        self.current_node = start_node
        
        # This is our "Known Map"
        self.visited_edges = set()
        self.visited_nodes = {start_node} # Start with the start node
        
        # This is just a log of all moves for debugging/drawing
        self.path_history_log = [start_node] 
        
        self.exploration_complete = False
        self.target_found_once = False # To print message just once

    def get_unexplored_neighbors(self, node):
        """Finds 'orange' stubs for a *specific* node."""
        unexplored = []
        for neighbor in self.graph.neighbors(node):
            edge = tuple(sorted((node, neighbor)))
            if edge not in self.visited_edges:
                unexplored.append(neighbor)
        return unexplored

    def find_all_frontiers(self):
        """Finds all nodes *on the known map* that have an 'orange' stub."""
        frontiers = set()
        # We only need to check nodes we've already visited
        for node in self.visited_nodes:
            if self.get_unexplored_neighbors(node):
                # This node has an unexplored path, it's a frontier
                frontiers.add(node)
        return frontiers

    def update_position(self, next_node):
        """Moves the robot and updates the known map."""
        self.visited_edges.add(tuple(sorted((self.current_node, next_node))))
        self.visited_nodes.add(next_node)
        self.current_node = next_node
        self.path_history_log.append(next_node) # Append to the log

    def move_to_node(self, target_node):
        """Moves one step along the shortest path to a target node."""
        if self.current_node == target_node:
            return # We're already there

        # Create a graph of *only* the known map
        known_map = nx.Graph()
        known_map.add_nodes_from(self.visited_nodes)
        known_map.add_edges_from(self.visited_edges)
        
        try:
            # Get the *full* path
            path = nx.shortest_path(
                known_map,
                source=self.current_node,
                target=target_node
            )
            if len(path) > 1:
                # Take the first step
                next_step = path[1]
                self.update_position(next_step)
        except nx.NetworkXNoPath:
            # This should not happen if logic is correct
            print(f"Error: Cannot find path from {self.current_node} to {target_node}")
        except nx.NodeNotFound:
            # This can happen if the target_node is not yet in visited_nodes
            # which it shouldn't be. Wait, target_node is a frontier, so it
            # *must* be in visited_nodes. This is a failsafe.
            print(f"Error: Node {target_node} not in known map.")


    def move(self):
        """
        Main move function. Uses the "Nearest Frontier" algorithm.
        """
        if self.exploration_complete:
            return

        # Check if we just found the target (for printing)
        if (self.current_node == self.target_node and 
            not self.target_found_once):
            print("--- Target Found! Continuing to explore... ---")
            self.target_found_once = True
            
        # 1. (Priority 1) Are there unexplored paths *right here*?
        unexplored_here = self.get_unexplored_neighbors(self.current_node)
        if unexplored_here:
            # Yes. Explore one of them. This is the most efficient move.
            next_node_to_explore = unexplored_here[0]
            self.update_position(next_node_to_explore)
            return

        # 2. (Priority 2) No local paths. Find the *nearest* other frontier.
        
        # 2a. Find all frontiers on the map.
        all_frontiers = self.find_all_frontiers()
        
        if not all_frontiers:
            # 3. No frontiers left anywhere. Exploration is done. Go home.
            if self.current_node == self.start_node:
                print("Exploration complete! Returned to start.")
                self.exploration_complete = True
            else:
                # Travel back to the start node
                self.move_to_node(self.start_node)
            return
            
        # 2b. We have frontiers. Find the nearest one.
        known_map = nx.Graph()
        known_map.add_nodes_from(self.visited_nodes)
        known_map.add_edges_from(self.visited_edges)
        
        nearest_frontier = None
        min_dist = float('inf')
        
        for frontier_node in all_frontiers:
            try:
                # Calculate path on the *known map*
                path_len = nx.shortest_path_length(
                    known_map, 
                    source=self.current_node, 
                    target=frontier_node
                )
                if path_len < min_dist:
                    min_dist = path_len
                    nearest_frontier = frontier_node
            except nx.NetworkXNoPath:
                # Should not happen in a connected known map
                pass 
        
        if nearest_frontier is not None:
            # 2c. We have a target. Move one step towards it.
            self.move_to_node(nearest_frontier)
        else:
            # Failsafe
            print("Error: No reachable frontiers.")
            self.exploration_complete = True


# --- 5. Run the Simulation (MODIFIED) ---
# Now uses robot.visited_nodes for drawing

if __name__ == "__main__":
    
    plt.ion() 
    fig, ax = plt.subplots(figsize=(12, 12)) # Create the animation window

    robot = Robot(G, START_NODE, TARGET_NODE)

    shortest_path_nodes = None 

    max_steps = 500
    for i in range(max_steps):
        
        # --- (MODIFIED) ---
        # We now pass the robot's set of known nodes directly
        visited_nodes_set = robot.visited_nodes
        
        draw_maze(
            G, 
            robot.current_node, 
            robot.visited_edges, 
            visited_nodes_set, # <-- Pass the correct set
            optimal_path=None, 
            ax=ax
        )
        if PRINT_SPEED > 0:
            plt.pause(PRINT_SPEED)
        
        if robot.exploration_complete:
            print(f"Simulation finished in {i} steps.")
            
            print("Calculating shortest path...")
            shortest_path_nodes = nx.shortest_path(
                G, source=START_NODE, target=TARGET_NODE
            )
            print(f"Shortest path: {shortest_path_nodes}")
            
            break # Exit the simulation loop
            
        robot.move()

    if not robot.exploration_complete:
        print(f"Simulation stopped after {max_steps} steps. Map not fully explored.")

    # --- (Unchanged) Final Draw Logic ---
    
    plt.ioff() 
    plt.close(fig) 

    print("Simulation complete. Displaying final path...")
    
    fig_final, ax_final = plt.subplots(figsize=(12, 12))
    
    visited_nodes_set_final = robot.visited_nodes
    
    draw_maze(
        G, 
        robot.current_node,           
        robot.visited_edges,          
        visited_nodes_set_final,      
        optimal_path=shortest_path_nodes, 
        ax=ax_final                   
    )
    plt.show()