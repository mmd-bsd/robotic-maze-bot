import networkx as nx
import matplotlib.pyplot as plt
import math
from enum import Enum 

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
# (Unchanged)
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
    
    # 4. Draw the Optimal Path
    if optimal_path:
        optimal_edges = list(zip(optimal_path[:-1], optimal_path[1:]))
        nx.draw_networkx_edges(
            graph, pos, ax=ax,
            edgelist=optimal_edges,
            edge_color='magenta',
            width=6
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
    ax.set_title("Maze Simulator - Full Exploration (DFS) + Optimal Path")
    ax.legend()
    ax.axis('equal')


# --- 4. Robot Class (SIMPLIFIED - Unified DFS Algorithm) ---

class Robot:
    def __init__(self, graph, start_node, target_node):
        self.graph = graph
        self.start_node = start_node
        self.target_node = target_node
        
        self.current_node = start_node
        (start_x, start_y) = graph.nodes[start_node]['pos']
        self.previous_pos = (start_x, start_y - 20) # Virtual pos for initial direction
        
        self.visited_edges = set()
        self.path_history = [start_node] # DFS Stack
        
        self.exploration_complete = False
        self.target_found_once = False # To print message just once

    def get_unexplored_neighbors(self):
        """Helper function to find 'orange' paths from current node."""
        unexplored = []
        for neighbor in self.graph.neighbors(self.current_node):
            edge = tuple(sorted((self.current_node, neighbor)))
            if edge not in self.visited_edges:
                unexplored.append(neighbor)
        return unexplored

    def move(self):
        """
        Main move function. Uses a single, unified DFS exploration
        algorithm from start to finish.
        """
        if self.exploration_complete:
            return

        # Check if we just found the target (for printing)
        if (self.current_node == self.target_node and 
            not self.target_found_once):
            print("--- Target Found! Continuing to explore... ---")
            self.target_found_once = True
            
        # 1. Check for unexplored paths from the current node
        unexplored_list = self.get_unexplored_neighbors()
        
        if len(unexplored_list) > 0:
            # 2. If yes: Explore the first available one
            # (This is the "DFS" part - go deep)
            next_node = unexplored_list[0]
            self.update_position(next_node)
        
        else:
            # 3. If no: This node is fully explored. Backtrack.
            
            # Check if we are back at the start and the start is also fully explored
            if self.current_node == self.start_node:
                print("Exploration complete! Returned to start.")
                self.exploration_complete = True
                return

            # Pop the stack to backtrack
            if len(self.path_history) > 1:
                _ = self.path_history.pop() # Pop current node
                next_node = self.path_history[-1] # Get new current node (backtrack)
                
                # Update position *without* calling update_position
                # to avoid messing with the stack
                self.previous_pos = self.graph.nodes[self.current_node]['pos']
                self.current_node = next_node
            else:
                self.exploration_complete = True

    def update_position(self, next_node):
        """
        Helper function to move robot and (CRITICALLY)
        manage the path_history stack correctly.
        """
        # (This is the logic from the previous fix)
        self.visited_edges.add(tuple(sorted((self.current_node, next_node))))
        self.previous_pos = self.graph.nodes[self.current_node]['pos']
        self.current_node = next_node
        
        # Check if we are backtracking (moving to the node we just came from)
        if len(self.path_history) > 1 and next_node == self.path_history[-2]:
            # We are backtracking, so POP the stack
            self.path_history.pop()
        else:
            # We are exploring a new node, so PUSH to the stack
            self.path_history.append(self.current_node)


# --- 5. Run the Simulation ---
# (Unchanged)

if __name__ == "__main__":
    
    plt.ion() 
    fig, ax = plt.subplots(figsize=(12, 12)) # Create the animation window

    robot = Robot(G, START_NODE, TARGET_NODE)

    shortest_path_nodes = None 

    max_steps = 500
    for i in range(max_steps):
        
        visited_nodes_set = set(robot.path_history)
        
        draw_maze(
            G, 
            robot.current_node, 
            robot.visited_edges, 
            visited_nodes_set,
            optimal_path=None, 
            ax=ax
        )
        
        plt.pause(0.5) # Adjust animation speed
        
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

    # --- (BUG FIX) Final Draw Logic ---
    
    plt.ioff() # Turn off interactive mode
    plt.close(fig) # Close the animation window

    print("Simulation complete. Displaying final path...")
    
    # Create a brand new, clean figure for the final plot
    fig_final, ax_final = plt.subplots(figsize=(12, 12))
    
    visited_nodes_set_final = set(robot.path_history)
    
    draw_maze(
        G, 
        robot.current_node,           # Robot is at node 0
        robot.visited_edges,          # All explored edges are blue
        visited_nodes_set_final,      # No more orange stubs
        optimal_path=shortest_path_nodes, # The pink path!
        ax=ax_final                   # Draw on the new, clean axis
    )
    plt.show() # Show the final, static plot