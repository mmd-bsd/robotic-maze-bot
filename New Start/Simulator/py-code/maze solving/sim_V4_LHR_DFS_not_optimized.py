import networkx as nx
import matplotlib.pyplot as plt
import math
from enum import Enum # Added for state management

# --- 1. Define Maze Structure (User-corrected Map) ---
# This is the accurate map you provided.

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

# Define the edges (paths) connecting the nodes
edges_list = [
    (0, 1), (1, 2), (1, 21), (2, 3), (2, 6), (3, 4),
    (3, 10), (3, 15), (4, 5), (4, 7), (4, 20), (7, 8),
    (9, 10), (9, 12), (10, 11), (11, 13), (12, 13),
    (14, 15), (14, 17), (15, 16), (16, 19), (17, 18),
    (17, 24), (18, 25), (19, 20), (19, 22), (22, 23),
    (22, 26), (24, 25), (25, 26),
]

# Define special nodes
START_NODE = 0
TARGET_NODE = 22

# --- 2. Create Graph ---
G = nx.Graph()
G.add_nodes_from(nodes_positions.keys())
G.add_edges_from(edges_list)
for node, pos in nodes_positions.items():
    G.nodes[node]['pos'] = pos


# --- 3. Visualization Function ---
# This function includes the logic for drawing "orange stubs"

def draw_maze(graph, robot_node=None, visited_edges=None, visited_nodes=None, ax=None):
    """
    Draws the current state of the maze, robot, visited paths,
    and detected but unexplored stubs.
    """
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

    # 3. Draw unexplored stubs (orange dashed lines)
    STUB_LENGTH = 10.0
    
    for u in visited_nodes:
        (x1, y1) = graph.nodes[u]['pos']
        
        for v in graph.neighbors(u):
            edge = tuple(sorted((u, v)))
            
            # Check if this edge was NOT traversed
            if edge not in visited_edges:
                (x2, y2) = graph.nodes[v]['pos']
                
                dx = x2 - x1
                dy = y2 - y1
                mag = math.sqrt(dx**2 + dy**2)
                
                if mag > 0: # Avoid division by zero
                    ux = dx / mag
                    uy = dy / mag
                    
                    x_end = x1 + STUB_LENGTH * ux
                    y_end = y1 + STUB_LENGTH * uy
                    
                    ax.plot(
                        [x1, x_end],    # [x_start, x_end]
                        [y1, y_end],    # [y_start, y_end]
                        color='orange',
                        linewidth=3,
                        linestyle='--' # Dashed line
                    )

    # 4. Draw all nodes
    nx.draw_networkx_nodes(
        graph, pos, ax=ax,
        node_color='lightblue', node_size=400
    )
    
    # 5. Draw Start, Target, and Robot nodes
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
    ax.set_title("Maze Simulator - Full Exploration (DFS)")
    ax.legend()
    ax.axis('equal')


# --- 4. Robot Class (Corrected Logic) ---

class RobotState(Enum):
    FINDING_TARGET = 1 # Use Left-Hand Rule
    COMPLETING_MAP = 2 # Use DFS/Backtracking

class Robot:
    def __init__(self, graph, start_node, target_node):
        self.graph = graph
        self.start_node = start_node
        self.target_node = target_node
        
        self.current_node = start_node
        (start_x, start_y) = graph.nodes[start_node]['pos']
        # Virtual pos for initial direction
        self.previous_pos = (start_x, start_y - 20) 
        
        self.visited_edges = set()
        # path_history acts as our Stack for DFS, must be updated in *all* phases
        self.path_history = [start_node] 
        
        self.state = RobotState.FINDING_TARGET
        self.exploration_complete = False
        self.target_found_once = False # To prevent state-switching loop

    def get_relative_directions(self):
        """Calculates relative directions using angles."""
        neighbors = list(self.graph.neighbors(self.current_node))
        (cx, cy) = self.graph.nodes[self.current_node]['pos']
        (px, py) = self.previous_pos

        angle_in = math.atan2(cy - py, cx - px)
        relative_map = {'left': None, 'front': None, 'right': None, 'back': None}
        epsilon = 0.01 

        for neighbor in neighbors:
            (nx, ny) = self.graph.nodes[neighbor]['pos']
            angle_out = math.atan2(ny - cy, nx - cx)
            delta_angle = angle_out - angle_in
            delta_angle = math.atan2(math.sin(delta_angle), math.cos(delta_angle))

            if abs(delta_angle - (math.pi / 2)) < epsilon:
                relative_map['left'] = neighbor
            elif abs(delta_angle) < epsilon:
                relative_map['front'] = neighbor
            elif abs(delta_angle - (-math.pi / 2)) < epsilon:
                relative_map['right'] = neighbor
            elif abs(abs(delta_angle) - math.pi) < epsilon:
                relative_map['back'] = neighbor
        
        return relative_map

    def move_left_hand_rule(self):
        """Applies the Left-Hand Rule logic."""
        directions = self.get_relative_directions()
        
        next_node = None
        if directions['left'] is not None:
            next_node = directions['left']
        elif directions['front'] is not None:
            next_node = directions['front']
        elif directions['right'] is not None:
            next_node = directions['right']
        elif directions['back'] is not None:
            next_node = directions['back']
        else:
            return # Trapped

        # update_position will add this to path_history
        self.update_position(next_node)

    def get_unexplored_neighbors(self):
        """Helper function to find 'orange' paths from current node."""
        unexplored = []
        for neighbor in self.graph.neighbors(self.current_node):
            edge = tuple(sorted((self.current_node, neighbor)))
            if edge not in self.visited_edges:
                unexplored.append(neighbor)
        return unexplored

    def move_dfs_exploration(self):
        """Applies the DFS/Backtracking logic."""
        
        unexplored_list = self.get_unexplored_neighbors()
        
        if len(unexplored_list) > 0:
            # 1. If yes: Explore the first available one
            next_node = unexplored_list[0]
            # update_position will add it to the stack
            self.update_position(next_node)
        
        else:
            # 2. If no: This node is fully explored. Backtrack.
            
            # Check if we are back at the start and the start is also fully explored
            if self.current_node == self.start_node:
                print("Exploration complete! Returned to start.")
                self.exploration_complete = True
                return

            # Pop the stack to backtrack
            if len(self.path_history) > 1:
                _ = self.path_history.pop() # Remove current node from stack
                next_node = self.path_history[-1] # Get new current node (backtrack)
                
                # We do NOT call update_position, as we are not exploring.
                # We just "teleport" back, ready for the next move.
                self.previous_pos = self.graph.nodes[self.current_node]['pos']
                self.current_node = next_node
            else:
                # Should not be reached if logic is correct
                self.exploration_complete = True


    def update_position(self, next_node):
        """
        Helper function to move robot, update state, 
        and (CRITICALLY) update the path_history stack.
        """
        self.visited_edges.add(tuple(sorted((self.current_node, next_node))))
        self.previous_pos = self.graph.nodes[self.current_node]['pos']
        self.current_node = next_node
        
        # --- THIS IS THE FIX ---
        # Add the new node to the path history (stack)
        # This ensures the stack is built even during the LHR phase.
        self.path_history.append(self.current_node)
        # --- END FIX ---
        
    def move(self):
        """Main move function, acts as a state manager."""
        
        if self.exploration_complete:
            return

        # Check for state transition
        if (self.state == RobotState.FINDING_TARGET and 
            self.current_node == self.target_node and 
            not self.target_found_once):
            
            print("--- Target Found! Switching to Full Exploration Mode ---")
            self.state = RobotState.COMPLETING_MAP
            self.target_found_once = True # Prevent this from running again
        
        # Execute move based on state
        if self.state == RobotState.FINDING_TARGET:
            self.move_left_hand_rule()
        
        elif self.state == RobotState.COMPLETING_MAP:
            self.move_dfs_exploration()


# --- 5. Run the Simulation ---
if __name__ == "__main__":
    
    plt.ion() # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(12, 12))

    robot = Robot(G, START_NODE, TARGET_NODE)

    max_steps = 500 # Increased steps for full exploration
    for i in range(max_steps):
        
        # Draw the current state
        # We get the visited_nodes from the *full* path history
        visited_nodes_set = set(robot.path_history)
        
        draw_maze(
            G, 
            robot.current_node, 
            robot.visited_edges, 
            visited_nodes_set, # <-- This now works correctly
            ax
        )
        
        plt.pause(0.5) # Adjust animation speed here
        
        # Check for simulation stop condition
        if robot.exploration_complete:
            print(f"Simulation finished in {i} steps.")
            print(f"Final path history (stack): {robot.path_history}")
            break
            
        robot.move()

    if not robot.exploration_complete:
        print(f"Simulation stopped after {max_steps} steps. Map not fully explored.")

    print("Simulation complete. Close the plot window to exit.")
    plt.ioff()
    
    # Draw one final time to show the complete map
    visited_nodes_set = set(robot.path_history)
    draw_maze(G, robot.current_node, robot.visited_edges, visited_nodes_set, ax)
    plt.show()