import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import sys
import math

# --- Configuration ---
GRID_SIZE = 20  # Snap clicks to a 20x20 grid
MAX_X = 300
MAX_Y = 300
# ---------------------

class MazeEditorV3:
    def __init__(self, grid_size):
        self.grid_size = grid_size
        
        # We store edges by their actual coordinates (pos1, pos2)
        self.edges_by_pos = set()
        
        self.start_pos = None
        self.target_pos = None
        self.selected_pos = None # Stores (x, y) of the first click

        # Setup the plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        # (FIX for Qt backend)
        if hasattr(self.fig.canvas, 'setWindowTitle'):
            self.fig.canvas.setWindowTitle('Maze Editor v3 (Edge-Based)')
        elif hasattr(self.fig.canvas, 'set_window_title'):
            self.fig.canvas.set_window_title('Maze Editor v3 (Edge-Based)')
        
        # Connect event handlers
        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_key = self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        self.draw_editor()

    def snap_to_grid(self, x, y):
        """Rounds (x, y) coordinates to the nearest grid point."""
        snapped_x = int(np.round(x / self.grid_size) * self.grid_size)
        snapped_y = int(np.round(y / self.grid_size) * self.grid_size)
        return (snapped_x, snapped_y)

    def on_click(self, event):
        """Handles mouse clicks for drawing edges."""
        if event.inaxes != self.ax: return
        if event.button != 1: return # Only react to left-click
        
        x, y = event.xdata, event.ydata
        if x is None or y is None: return

        clicked_pos = self.snap_to_grid(x, y)

        if self.selected_pos is None:
            # --- First Click: Select starting point ---
            self.selected_pos = clicked_pos
            print(f"Position {clicked_pos} selected as first point.")
        
        elif self.selected_pos == clicked_pos:
            # --- Click on same point: Deselect ---
            print(f"Position {self.selected_pos} deselected.")
            self.selected_pos = None
            
        else:
            # --- Second Click: Create Edge ---
            pos1 = self.selected_pos
            pos2 = clicked_pos
            
            # Sort the tuple to ensure (A, B) is the same as (B, A)
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
        """Handles key presses."""
        
        # We only care about keys if a position is selected
        if self.selected_pos is None and event.key not in ['p', 'q']:
             # If 'p' (print) or 'q' (quit) is pressed, let it proceed
            if event.key not in ['p', 'q']:
                print("Click on a grid intersection first to select it.")
                return

        # --- 's' key: Set Start Node ---
        if event.key == 's':
            if self.selected_pos:
                self.start_pos = self.selected_pos
                print(f"Position {self.selected_pos} set as START.")
                self.selected_pos = None # Deselect after setting
            
        # --- 't' key: Set Target Node ---
        if event.key == 't':
            if self.selected_pos:
                self.target_pos = self.selected_pos
                print(f"Position {self.selected_pos} set as TARGET.")
                self.selected_pos = None # Deselect after setting

        # --- 'd' key: Delete Node and Edges ---
        if event.key == 'd':
            if self.selected_pos:
                pos_to_delete = self.selected_pos
                
                # Find all edges connected to this position
                edges_to_remove = set()
                for edge in self.edges_by_pos:
                    if pos_to_delete in edge:
                        edges_to_remove.add(edge)
                
                # Remove the edges
                self.edges_by_pos.difference_update(edges_to_remove)
                
                # Clear special status if this pos was start/target
                if self.start_pos == pos_to_delete: self.start_pos = None
                if self.target_pos == pos_to_delete: self.target_pos = None
                
                print(f"Position {pos_to_delete} and {len(edges_to_remove)} connected edges deleted.")
                self.selected_pos = None
            
        # --- 'p' key: Print Code ---
        if event.key == 'p':
            self.generate_code()
            
        # --- 'q' key: Quit ---
        if event.key == 'q':
            plt.close(self.fig)
            print("Editor closed.")

        self.draw_editor()

    def generate_code(self):
        """
        COMPILER: Processes the raw coordinates and edges,
        assigns Node IDs, and prints the final code.
        """
        print("\n--- (Copy the code below) ---")
        
        # 1. Collect all unique positions (nodes)
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

        # 2. Create a mapping from position (x,y) to a Node ID
        # Sort positions to make IDs consistent (optional but nice)
        sorted_positions = sorted(list(all_positions))
        pos_to_id = {pos: i for i, pos in enumerate(sorted_positions)}
        
        # 3. Generate nodes_positions
        print("\nnodes_positions = {")
        for pos, node_id in pos_to_id.items():
            print(f"    {node_id}: {pos},")
        print("}")
        
        # 4. Generate edges_list
        print("\nedges_list = [")
        for (pos1, pos2) in self.edges_by_pos:
            id1 = pos_to_id[pos1]
            id2 = pos_to_id[pos2]
            print(f"    ({id1}, {id2}),")
        print("]")
        
        # 5. Generate START_NODE and TARGET_NODE
        start_id = pos_to_id.get(self.start_pos, 'None')
        target_id = pos_to_id.get(self.target_pos, 'None')
        
        print(f"\nSTART_NODE = {start_id}")
        print(f"TARGET_NODE = {target_id}")
        print("--- (End of code) ---\n")

    def draw_editor(self):
        """Redraws the entire editor state."""
        self.ax.clear()
        
        # Set up grid
        self.ax.set_xlim(-self.grid_size, MAX_X + self.grid_size)
        self.ax.set_ylim(-self.grid_size, MAX_Y + self.grid_size)
        self.ax.set_xticks(range(0, MAX_X + 1, self.grid_size))
        self.ax.set_yticks(range(0, MAX_Y + 1, self.grid_size))
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.ax.set_axisbelow(True)
        self.ax.set_aspect('equal')
        self.ax.set_title("Edge Editor | Click-Click to Draw | [P]rint | [S]tart | [T]arget | [D]elete | [Q]uit")
        
        # 1. Draw all edges
        for (pos1, pos2) in self.edges_by_pos:
            x_values = [pos1[0], pos2[0]]
            y_values = [pos1[1], pos2[1]]
            self.ax.plot(x_values, y_values, color='black', linewidth=3)
        
        # 2. Draw all nodes (intersections)
        all_nodes_pos = set()
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
    print("--- Maze Editor v3 (Edge-Based) Help ---")
    print(" - Left Click: Select first grid point (yellow).")
    print(" - Second Left Click: Draw edge from first point to second.")
    print(" - ---")
    print(" - Click a point, THEN press a key:")
    print(" - Key 's': Set selected point as START (green).")
    print(" - Key 't': Set selected point as TARGET (black).")
    print(" - Key 'd': Delete selected point and all connected edges.")
    print(" - ---")
    print(" - Key 'p': Print the map code to the console.")
    print(" - Key 'q': Quit the editor.")
    print("------------------------------------------")
    
    # Needs networkx, matplotlib, numpy
    # pip install networkx matplotlib numpy
    
    editor = MazeEditorV3(grid_size=GRID_SIZE)
    plt.show()