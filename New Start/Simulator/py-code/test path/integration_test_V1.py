import networkx as nx
import matplotlib.pyplot as plt
import math
from enum import Enum 

PRINT_SPEED = 0.1 #seconds

nodes_positions = {
    0: (0, 80),
    1: (0, 180),
    2: (0, 280),
    3: (40, 20),
    4: (40, 40),
    5: (40, 80),
    6: (40, 260),
    7: (40, 280),
    8: (60, 20),
    9: (60, 40),
    10: (60, 80),
    11: (60, 180),
    12: (60, 220),
    13: (60, 240),
    14: (60, 280),
    15: (60, 300),
    16: (80, 140),
    17: (80, 180),
    18: (80, 200),
    19: (100, 0),
    20: (100, 40),
    21: (100, 60),
    22: (100, 140),
    23: (100, 160),
    24: (100, 180),
    25: (100, 200),
    26: (120, 100),
    27: (120, 140),
    28: (120, 160),
    29: (120, 180),
    30: (120, 240),
    31: (120, 280),
    32: (140, 0),
    33: (140, 40),
    34: (140, 80),
    35: (140, 100),
    36: (140, 120),
    37: (140, 240),
    38: (180, 20),
    39: (180, 40),
    40: (180, 60),
    41: (180, 100),
    42: (180, 220),
    43: (180, 280),
    44: (200, 80),
    45: (200, 100),
    46: (220, 220),
    47: (220, 240),
    48: (220, 280),
    49: (240, 60),
    50: (240, 80),
    51: (240, 100),
    52: (240, 180),
    53: (240, 200),
    54: (240, 240),
    55: (260, 60),
    56: (260, 80),
    57: (260, 100),
    58: (280, 60),
    59: (280, 80),
    60: (280, 100),
    61: (280, 120),
    62: (280, 140),
    63: (300, 80),
    64: (300, 140),
    65: (300, 200),
    66: (320, 80),
    67: (320, 140),
}

edges_list = [
    (13, 14),
    (11, 17),
    (1, 11),
    (26, 27),
    (22, 27),
    (0, 1),
    (34, 35),
    (41, 45),
    (47, 48),
    (66, 67),
    (43, 48),
    (53, 54),
    (63, 66),
    (26, 35),
    (50, 51),
    (56, 57),
    (13, 30),
    (6, 7),
    (35, 36),
    (57, 60),
    (30, 31),
    (40, 41),
    (23, 24),
    (64, 67),
    (62, 64),
    (21, 40),
    (10, 11),
    (64, 65),
    (8, 38),
    (31, 43),
    (19, 20),
    (2, 7),
    (32, 33),
    (42, 43),
    (58, 59),
    (16, 17),
    (47, 54),
    (38, 39),
    (24, 25),
    (28, 29),
    (8, 9),
    (16, 22),
    (18, 25),
    (4, 5),
    (53, 65),
    (3, 4),
    (55, 56),
    (59, 63),
    (39, 40),
    (36, 61),
    (17, 18),
    (27, 28),
    (55, 58),
    (42, 46),
    (21, 22),
    (49, 50),
    (4, 9),
    (46, 47),
    (61, 62),
    (29, 52),
    (50, 56),
    (14, 15),
    (7, 14),
    (5, 10),
    (44, 45),
    (52, 53),
    (0, 5),
    (30, 37),
    (23, 28),
    (44, 50),
    (63, 64),
    (12, 13),
    (59, 60),
    (14, 31),
]

START_NODE = 2
TARGET_NODE = 41

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
            edge_color='green',  # <-- رنگ سبز
            width=20,
            alpha=0.7         # <-- (FIX) 100% کدر (Opaque)
        )

    # 5. Draw all nodes
    nx.draw_networkx_nodes(
        graph, pos, ax=ax,
        node_color='lightblue', node_size=200
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