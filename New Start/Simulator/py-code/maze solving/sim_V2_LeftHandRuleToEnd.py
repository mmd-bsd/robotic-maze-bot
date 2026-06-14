import networkx as nx
import matplotlib.pyplot as plt
import math # Import the math library for angle calculations

# --- 1. Define Maze Structure (User-corrected Map) ---

# Define special nodes

START_NODE = 0
TARGET_NODE = 22

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



# --- 2. Create Graph ---
G = nx.Graph()
G.add_nodes_from(nodes_positions.keys())
G.add_edges_from(edges_list)
for node, pos in nodes_positions.items():
    G.nodes[node]['pos'] = pos

# --- 3. Visualization Function (Unchanged) ---

# --- 3. Visualization Function (Updated for Unexplored Stubs) ---

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

    # 3. Draw unexplored stubs (NEW SECTION)
    # We use 'orange' as it's more visible than 'yellow'
    STUB_LENGTH = 10.0
    
    for u in visited_nodes:
        (x1, y1) = graph.nodes[u]['pos']
        
        for v in graph.neighbors(u):
            edge = tuple(sorted((u, v)))
            
            # Check if this edge was NOT traversed
            if edge not in visited_edges:
                (x2, y2) = graph.nodes[v]['pos']
                
                # Calculate direction vector
                dx = x2 - x1
                dy = y2 - y1
                
                # Calculate magnitude (length)
                mag = math.sqrt(dx**2 + dy**2)
                
                if mag > 0: # Avoid division by zero
                    # Calculate unit vector
                    ux = dx / mag
                    uy = dy / mag
                    
                    # Calculate end point of the 10-unit stub
                    x_end = x1 + STUB_LENGTH * ux
                    y_end = y1 + STUB_LENGTH * uy
                    
                    # Draw the stub using matplotlib's plot
                    ax.plot(
                        [x1, x_end],    # [x_start, x_end]
                        [y1, y_end],    # [y_start, y_end]
                        color='orange',
                        linewidth=3,
                        linestyle='--' # Dashed line for "potential"
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
    
    ax.set_title("Maze Simulator - Left-Hand Rule (Angle-Based Logic)")
    ax.legend()
    ax.axis('equal')

# --- 4. Robot Class (BUG FIXED with Angle Logic) ---

class Robot:
    def __init__(self, graph, start_node, target_node):
        self.graph = graph
        self.start_node = start_node
        self.target_node = target_node
        
        self.current_node = start_node
        
        # Get start coordinates
        (start_x, start_y) = graph.nodes[start_node]['pos']
        
        # Virtual previous position "behind" the start node
        self.previous_pos = (start_x, start_y - 20) 
        
        self.visited_edges = set()
        self.path_history = [start_node]
        self.target_found = False

    def get_relative_directions(self):
        """
        Determines relative directions using angle calculations (atan2).
        This is robust to different path lengths.
        """
        neighbors = list(self.graph.neighbors(self.current_node))
        
        (cx, cy) = self.graph.nodes[self.current_node]['pos']
        (px, py) = self.previous_pos

        # Calculate the angle of the incoming path
        dx_in = cx - px
        dy_in = cy - py
        angle_in = math.atan2(dy_in, dx_in)

        relative_map = {'left': None, 'front': None, 'right': None, 'back': None}
        
        # A small tolerance for floating point comparisons
        epsilon = 0.01 

        for neighbor in neighbors:
            (nx, ny) = self.graph.nodes[neighbor]['pos']
            
            # Calculate the angle of the outgoing path
            dx_out = nx - cx
            dy_out = ny - cy
            angle_out = math.atan2(dy_out, dx_out)

            # Find the relative angle difference
            # We normalize the angle to be between -pi and +pi
            delta_angle = angle_out - angle_in
            delta_angle = math.atan2(math.sin(delta_angle), math.cos(delta_angle))

            # Classify based on the relative angle
            # Note: math.pi/2 is 90 degrees (Left)
            # Note: -math.pi/2 is -90 degrees (Right)
            
            if abs(delta_angle - (math.pi / 2)) < epsilon:
                relative_map['left'] = neighbor
            elif abs(delta_angle) < epsilon:
                relative_map['front'] = neighbor
            elif abs(delta_angle - (-math.pi / 2)) < epsilon:
                relative_map['right'] = neighbor
            elif abs(abs(delta_angle) - math.pi) < epsilon:
                relative_map['back'] = neighbor
        
        return relative_map

    def move(self):
        """
        Performs one step using the Left-Hand Rule. (Logic unchanged)
        """
        if self.current_node == self.target_node:
            self.target_found = True
            print(f"Target found at node {self.target_node}!")
            return

        directions = self.get_relative_directions()
        
        next_node = None
        
        # Apply Left-Hand Rule priority
        if directions['left'] is not None:
            # print(f"Node {self.current_node}: Turning LEFT to {directions['left']}")
            next_node = directions['left']
        elif directions['front'] is not None:
            # print(f"Node {self.current_node}: Going FRONT to {directions['front']}")
            next_node = directions['front']
        elif directions['right'] is not None:
            # print(f"Node {self.current_node}: Turning RIGHT to {directions['right']}")
            next_node = directions['right']
        elif directions['back'] is not None:
            # print(f"Node {self.current_node}: Turning BACK to {directions['back']}")
            next_node = directions['back'] # Dead end
        else:
            print("Error: Robot is trapped!")
            return

        self.visited_edges.add(tuple(sorted((self.current_node, next_node))))
        self.previous_pos = self.graph.nodes[self.current_node]['pos']
        self.current_node = next_node
        self.path_history.append(self.current_node)

# --- 5. Run the Simulation ---
if __name__ == "__main__":
    
    plt.ion() 
    fig, ax = plt.subplots(figsize=(12, 12))

    robot = Robot(G, START_NODE, TARGET_NODE)

    max_steps = 150 
    for i in range(max_steps):
        
        # --- MODIFIED CALL ---
        # Get the set of all nodes the robot has been to
        visited_nodes_set = set(robot.path_history)
        
        # Pass this set to the draw function
        draw_maze(
            G, 
            robot.current_node, 
            robot.visited_edges, 
            visited_nodes_set,  # <-- New parameter
            ax
        )
        # --- END MODIFIED CALL ---
        
        plt.pause(0.3) 
        
        if robot.target_found:
            print(f"Simulation finished in {i} steps.")
            print(f"Path taken: {robot.path_history}")
            break
            
        robot.move()

    if not robot.target_found:
        print(f"Simulation stopped after {max_steps} steps. Target not found.")

    print("Simulation complete. Close the plot window to exit.")
    plt.ioff()
    # Draw one final time to make sure the last step is shown
    visited_nodes_set = set(robot.path_history)
    draw_maze(G, robot.current_node, robot.visited_edges, visited_nodes_set, ax)
    plt.show()