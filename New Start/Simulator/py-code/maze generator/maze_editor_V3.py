import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import sys
import math

# --- Configuration ---
GRID_SIZE = 20  # Snap clicks to a 20x20 grid
MAX_X = 350
MAX_Y = 300
# ---------------------

def is_point_on_segment(point, end1, end2):
    """
    Checks if a point lies exactly on a line segment defined by two endpoints.
    Assumes points are tuples (x, y) and are collinear.
    """
    px, py = point
    x1, y1 = end1
    x2, y2 = end2

    # Check if point is within the bounding box of the segment
    in_x_bounds = (min(x1, x2) <= px <= max(x1, x2))
    in_y_bounds = (min(y1, y2) <= py <= max(y1, y2))
    
    return in_x_bounds and in_y_bounds

class MazeEditorV3:
    def __init__(self, grid_size):
        self.grid_size = grid_size
        self.edges_by_pos = set()
        self.start_pos = None
        self.target_pos = None
        self.selected_pos = None # Stores (x, y) of the first click

        # Setup the plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        if hasattr(self.fig.canvas, 'setWindowTitle'):
            self.fig.canvas.setWindowTitle('Maze Editor v3.1 (Edge Splitting)')
        elif hasattr(self.fig.canvas, 'set_window_title'):
            self.fig.canvas.set_window_title('Maze Editor v3.1 (Edge Splitting)')
        
        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_key = self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        self.draw_editor()

    def snap_to_grid(self, x, y):
        """Rounds (x, y) coordinates to the nearest grid point."""
        snapped_x = int(np.round(x / self.grid_size) * self.grid_size)
        snapped_y = int(np.round(y / self.grid_size) * self.grid_size)
        return (snapped_x, snapped_y)

    def on_click(self, event):
        """
        Handles mouse clicks with (NEW) edge-splitting logic.
        """
        if event.inaxes != self.ax: return
        if event.button != 1: return # Only react to left-click
        
        x, y = event.xdata, event.ydata
        if x is None or y is None: return

        clicked_pos = self.snap_to_grid(x, y)

        # --- (NEW) Smart Edge-Splitting Logic ---
        edge_to_split = None
        for (pos1, pos2) in self.edges_by_pos:
            # 1. Check for collinearity (cross product must be zero)
            # We are on a grid, so integer math is fine.
            dx1 = clicked_pos[0] - pos1[0]
            dy1 = clicked_pos[1] - pos1[1]
            dx2 = pos2[0] - pos1[0]
            dy2 = pos2[1] - pos1[1]
            
            # Check if point is not one of the endpoints
            if clicked_pos == pos1 or clicked_pos == pos2:
                continue

            # Cross product
            is_collinear = (dx1 * dy2 - dy1 * dx2) == 0
            
            if is_collinear and is_point_on_segment(clicked_pos, pos1, pos2):
                edge_to_split = (pos1, pos2)
                break
        
        if edge_to_split:
            # 2. Split the edge
            pos1, pos2 = edge_to_split
            self.edges_by_pos.remove(edge_to_split)
            self.edges_by_pos.add(tuple(sorted((pos1, clicked_pos))))
            self.edges_by_pos.add(tuple(sorted((clicked_pos, pos2))))
            print(f"Split edge {pos1}-{pos2} at new node {clicked_pos}.")
            
            # 3. Automatically select the new node
            self.selected_pos = clicked_pos
            self.draw_editor()
            return
        # --- End of New Logic ---

        # --- (Original V3) Edge Drawing Logic ---
        if self.selected_pos is None:
            # First Click: Select starting point
            self.selected_pos = clicked_pos
            print(f"Position {clicked_pos} selected as first point.")
        
        elif self.selected_pos == clicked_pos:
            # Click on same point: Deselect
            print(f"Position {self.selected_pos} deselected.")
            self.selected_pos = None
            
        else:
            # Second Click: Create Edge
            pos1 = self.selected_pos
            pos2 = clicked_pos
            
            edge = tuple(sorted((pos1, pos2)))
            
            if edge not in self.edges_by_pos:
                self.edges_by_pos.add(edge)
                print(f"Edge created between {pos1} and {pos2}.")
            else:
                print("Edge already exists.")
            
            # Reset selection
            self.selected_pos = None

        self.draw_editor()

    def on_key_press(self, event):
        """Handles key presses. (Unchanged from V3)"""
        
        if self.selected_pos is None and event.key not in ['p', 'q']:
             if event.key not in ['p', 'q']:
                print("Click on a grid intersection first to select it.")
                return

        if event.key == 's':
            if self.selected_pos:
                self.start_pos = self.selected_pos
                print(f"Position {self.selected_pos} set as START.")
                self.selected_pos = None 
            
        if event.key == 't':
            if self.selected_pos:
                self.target_pos = self.selected_pos
                print(f"Position {self.selected_pos} set as TARGET.")
                self.selected_pos = None 

        if event.key == 'd':
            if self.selected_pos:
                pos_to_delete = self.selected_pos
                
                edges_to_remove = set()
                for edge in self.edges_by_pos:
                    if pos_to_delete in edge:
                        edges_to_remove.add(edge)
                
                self.edges_by_pos.difference_update(edges_to_remove)
                
                if self.start_pos == pos_to_delete: self.start_pos = None
                if self.target_pos == pos_to_delete: self.target_pos = None
                
                print(f"Position {pos_to_delete} and {len(edges_to_remove)} connected edges deleted.")
                self.selected_pos = None
            
        if event.key == 'p':
            self.generate_code()
            
        if event.key == 'q':
            plt.close(self.fig)
            print("Editor closed.")

        self.draw_editor()

    def generate_code(self):
        """
        COMPILER: Processes the raw coordinates and edges,
        assigns Node IDs, and prints the final code.
        (Unchanged from V3)
        """
        print("\n--- (Copy the code below) ---")
        
        all_positions = set()
        if self.start_pos: all_positions.add(self.start_pos)
        if self.target_pos: all_positions.add(self.target_pos)
        for (pos1, pos2) in self.edges_by_pos:
            all_positions.add(pos1)
            all_positions.add(pos2)
            
        if not all_positions:
            print("Map is empty. Nothing to generate.")
            print("--- (End of code) ---\n")
            return

        sorted_positions = sorted(list(all_positions))
        pos_to_id = {pos: i for i, pos in enumerate(sorted_positions)}
        
        print("\nnodes_positions = {")
        for pos, node_id in pos_to_id.items():
            print(f"    {node_id}: {pos},")
        print("}")
        
        print("\nedges_list = [")
        for (pos1, pos2) in self.edges_by_pos:
            id1 = pos_to_id[pos1]
            id2 = pos_to_id[pos2]
            print(f"    ({id1}, {id2}),")
        print("]")
        
        start_id = pos_to_id.get(self.start_pos, 'None')
        target_id = pos_to_id.get(self.target_pos, 'None')
        
        print(f"\nSTART_NODE = {start_id}")
        print(f"TARGET_NODE = {target_id}")
        print("--- (End of code) ---\n")

    def draw_editor(self):
        """Redraws the entire editor state. (Unchanged from V3)"""
        self.ax.clear()
        
        # Set up grid
        self.ax.set_xlim(-self.grid_size, MAX_X + self.grid_size)
        self.ax.set_ylim(-self.grid_size, MAX_Y + self.grid_size)
        self.ax.set_xticks(range(0, MAX_X + 1, self.grid_size))
        self.ax.set_yticks(range(0, MAX_Y + 1, self.grid_size))
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.ax.set_axisbelow(True)
        self.ax.set_aspect('equal')
        self.ax.set_title("Edge Editor (v3.1) | [P]rint | [S]tart | [T]arget | [D]elete | [Q]uit")
        
        # 1. Draw all edges
        for (pos1, pos2) in self.edges_by_pos:
            x_values = [pos1[0], pos2[0]]
            y_values = [pos1[1], pos2[1]]
            self.ax.plot(x_values, y_values, color='black', linewidth=3)
        
        # 2. Draw all nodes (intersections)
        all_nodes_pos = set()
        if self.start_pos: all_nodes_pos.add(self.start_pos)
        if self.target_pos: all_nodes_pos.add(self.target_pos)
        for (pos1, pos2) in self.edges_by_pos:
            all_nodes_pos.add(pos1)
            all_nodes_pos.add(pos2)
            
        for pos in all_nodes_pos:
            self.ax.plot(pos[0], pos[1], 'o', markersize=10, color='lightblue')
            
        # 3. Highlight special points
        if self.start_pos:
            self.ax.plot(self.start_pos[0], self.start_pos[1], 'o', markersize=12, color='green')
        if self.target_pos:
            self.ax.plot(self.target_pos[0], self.target_pos[1], 'o', markersize=12, color='black')
        if self.selected_pos:
            self.ax.plot(self.selected_pos[0], self.selected_pos[1], 'o', 
                         markersize=12, color='yellow', 
                         markeredgecolor='orange', markeredgewidth=2)

        plt.draw()


# --- Main execution ---
if __name__ == "__main__":
    print("--- Maze Editor v3.1 (Edge-Splitting) Help ---")
    print(" - Left Click: Select first grid point (yellow).")
    print(" - Second Left Click: Draw edge from first point to second.")
    print(" - Left Click ON an existing edge: Splits the edge at that point.")
    print(" - ---")
    print(" - Click a point, THEN press a key:")
    print(" - Key 's': Set selected point as START (green).")
    print(" - Key 't': Set selected point as TARGET (black).")
    print(" - Key 'd': Delete selected point and all connected edges.")
    print(" - ---")
    print(" - Key 'p': Print the map code to the console.")
    print(" - Key 'q': Quit the editor.")
    print("------------------------------------------")
    
    editor = MazeEditorV3(grid_size=GRID_SIZE)
    plt.show()