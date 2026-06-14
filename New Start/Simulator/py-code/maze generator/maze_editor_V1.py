import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import sys

# --- Configuration ---
GRID_SIZE = 20  # Snap clicks to a 20x20 grid
MAX_X = 140
MAX_Y = 140
# ---------------------

class MazeEditor:
    def __init__(self, grid_size):
        self.grid_size = grid_size
        self.nodes_positions = {}
        self.edges_list = []
        self.start_node = None
        self.target_node = None
        self.selected_node = None
        self.next_node_id = 0

        # Setup the plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.setWindowTitle('Maze Editor')
        
        # Connect event handlers
        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.cid_key = self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        self.draw_editor()

    def snap_to_grid(self, x, y):
        """Rounds (x, y) coordinates to the nearest grid point."""
        snapped_x = np.round(x / self.grid_size) * self.grid_size
        snapped_y = np.round(y / self.grid_size) * self.grid_size
        return int(snapped_x), int(snapped_y)

    def get_node_at_pos(self, x, y):
        """Finds a node ID at a given (x, y) position."""
        for node_id, pos in self.nodes_positions.items():
            if pos == (x, y):
                return node_id
        return None

    def on_click(self, event):
        """Handles mouse clicks."""
        if event.inaxes != self.ax:
            return

        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return
            
        snapped_x, snapped_y = self.snap_to_grid(x, y)
        clicked_node_id = self.get_node_at_pos(snapped_x, snapped_y)

        # --- Left Click ---
        if event.button == 1:
            if clicked_node_id is not None:
                # Select an existing node
                self.selected_node = clicked_node_id
                print(f"Node {clicked_node_id} selected.")
            else:
                # Add a new node
                new_id = self.next_node_id
                self.nodes_positions[new_id] = (snapped_x, snapped_y)
                self.selected_node = new_id
                self.next_node_id += 1
                print(f"Node {new_id} created at {(snapped_x, snapped_y)} and selected.")

        # --- Right Click ---
        if event.button == 3:
            if self.selected_node is not None and clicked_node_id is not None:
                if self.selected_node != clicked_node_id:
                    # Add an edge
                    edge = tuple(sorted((self.selected_node, clicked_node_id)))
                    if edge not in self.edges_list:
                        self.edges_list.append(edge)
                        print(f"Edge created between {self.selected_node} and {clicked_node_id}.")
                    else:
                        print("Edge already exists.")
            # Deselect node
            elif self.selected_node is not None:
                print(f"Node {self.selected_node} deselected.")
                self.selected_node = None


        self.draw_editor()

    def on_key_press(self, event):
        """Handles key presses."""
        
        # --- 's' key: Set Start Node ---
        if event.key == 's':
            if self.selected_node is not None:
                self.start_node = self.selected_node
                print(f"Node {self.selected_node} set as START_NODE.")
            else:
                print("No node selected. Click a node to select it first.")

        # --- 't' key: Set Target Node ---
        if event.key == 't':
            if self.selected_node is not None:
                self.target_node = self.selected_node
                print(f"Node {self.selected_node} set as TARGET_NODE.")
            else:
                print("No node selected. Click a node to select it first.")
        
        # --- 'd' key: Delete Node ---
        if event.key == 'd':
            if self.selected_node is not None:
                node_to_delete = self.selected_node
                
                # Delete node
                del self.nodes_positions[node_to_delete]
                
                # Delete connected edges
                self.edges_list = [e for e in self.edges_list if node_to_delete not in e]
                
                # Clear special status
                if self.start_node == node_to_delete:
                    self.start_node = None
                if self.target_node == node_to_delete:
                    self.target_node = None
                
                self.selected_node = None
                print(f"Node {node_to_delete} and its edges deleted.")
            else:
                print("No node selected to delete.")

        # --- 'p' key: Print Code ---
        if event.key == 'p':
            self.generate_code()

        self.draw_editor()

    def generate_code(self):
        """Prints the generated map code to the console."""
        print("\n--- (Copy the code below) ---")
        
        print("\nnodes_positions = {")
        for node_id, pos in self.nodes_positions.items():
            print(f"    {node_id}: {pos},")
        print("}")
        
        print("\nedges_list = [")
        for edge in self.edges_list:
            print(f"    {edge},")
        print("]")
        
        print(f"\nSTART_NODE = {self.start_node}")
        print(f"TARGET_NODE = {self.target_node}")
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
        self.ax.set_title("Maze Editor | [P]rint Code | [S]et Start | [T]arget | [D]elete")
        
        # Create a graph to draw
        G = nx.Graph()
        G.add_nodes_from(self.nodes_positions.keys())
        G.add_edges_from(self.edges_list)
        pos = self.nodes_positions
        
        # Draw edges
        nx.draw_networkx_edges(G, pos, ax=self.ax, edge_color='grey', width=2)
        
        # Draw all nodes
        nx.draw_networkx_nodes(G, pos, ax=self.ax, node_color='lightblue', node_size=400)
        
        # Highlight special nodes
        if self.start_node is not None:
            nx.draw_networkx_nodes(G, pos, ax=self.ax, nodelist=[self.start_node], node_color='green', node_size=500)
        if self.target_node is not None:
            nx.draw_networkx_nodes(G, pos, ax=self.ax, nodelist=[self.target_node], node_color='black', node_size=500)
        if self.selected_node is not None:
            nx.draw_networkx_nodes(G, pos, ax=self.ax, nodelist=[self.selected_node], 
                                    node_color='yellow', node_size=500, 
                                    edgecolors='orange', linewidths=2) # Highlight selected

        # Draw labels
        nx.draw_networkx_labels(G, pos, ax=self.ax, font_color='black', font_size=9)

        plt.draw()


# --- Main execution ---
if __name__ == "__main__":
    print("--- Maze Editor Help ---")
    print(" - Left Click: Add new node, or select existing node.")
    print(" - Right Click: Connect two nodes (select one, right-click another), or deselect.")
    print(" - Key 's': Set selected node as START.")
    print(" - Key 't': Set selected node as TARGET.")
    print(" - Key 'd': Delete selected node.")
    print(" - Key 'p': Print the map code to the console.")
    print("------------------------")
    
    # Needs networkx, matplotlib, numpy
    # pip install networkx matplotlib numpy
    
    editor = MazeEditor(grid_size=GRID_SIZE)
    plt.show()