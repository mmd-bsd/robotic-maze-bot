# Robot Exploration Algorithm - Step-by-Step Description

## Algorithm Type: **Frontier-Based Exploration**

## Decision Logic Priority:
1. **Priority 1**: Explore unexplored neighbors directly from current node
2. **Priority 2**: If no unexplored neighbors, find and move to nearest frontier node
3. **Priority 3**: If no frontiers exist, return to start node

---

## First 15 Steps - Detailed Walkthrough

### **Step 1: Initialization**
- **Current Node**: 0 (position: {0, 0})
- **Visited Nodes**: {0}
- **Visited Edges**: None yet
- **Unexplored Neighbors**: [1] (node 0 connects to node 1)
- **Decision**: ✅ Take Priority 1 - Move to unexplored neighbor
- **Choice Made**: Node 1 (only option)
- **Movement**: Forward (F) - Robot heads toward node 1

---

### **Step 2: At Node 1 - First Decision Point!**
- **Current Node**: 1 (position: {0, 20})
- **Visited Nodes**: {0, 1}
- **Visited Edges**: (0-1)
- **Unexplored Neighbors**: [2, 21] ⚠️ **TWO CHOICES AVAILABLE!**
- **Decision Logic**: 
  ```c
  unexplored[0] = 2  // First in adjacency list
  unexplored[1] = 21 // Second in adjacency list
  ```
- **Choice Made**: Node 2 (takes `unexplored[0]` - **first option**)
- **How It Works**: The algorithm iterates through neighbors in adjacency list order and picks the first unexplored one
- **Movement**: Forward (F) - continues north to node 2

---

### **Step 3: At Node 2 - Another Decision Point**
- **Current Node**: 2 (position: {0, 40})
- **Visited Nodes**: {0, 1, 2}
- **Visited Edges**: (0-1), (1-2)
- **Unexplored Neighbors**: [3, 6] ⚠️ **TWO CHOICES**
- **Decision Logic**: Same as before - takes first in list
- **Choice Made**: Node 3 (first option)
- **Movement**: Forward (F) - continues north

---

### **Step 4: At Node 3 - Three-Way Choice**
- **Current Node**: 3 (position: {0, 60})
- **Visited Nodes**: {0, 1, 2, 3}
- **Visited Edges**: (0-1), (1-2), (2-3)
- **Unexplored Neighbors**: [4, 10, 15] ⚠️ **THREE CHOICES**
- **Decision Logic**: Takes first: `unexplored[0] = 4`
- **Choice Made**: Node 4
- **Movement**: Forward (F)

---

### **Step 5: At Node 4**
- **Current Node**: 4 (position: {0, 100})
- **Visited Nodes**: {0, 1, 2, 3, 4}
- **Visited Edges**: (0-1), (1-2), (2-3), (3-4)
- **Unexplored Neighbors**: [5, 7, 20] ⚠️ **THREE CHOICES**
- **Choice Made**: Node 5 (first option)
- **Movement**: Forward (F)

---

### **Step 6: At Node 5 - DEAD END! Priority 2 Activated**
- **Current Node**: 5 (position: {0, 120})
- **Visited Nodes**: {0, 1, 2, 3, 4, 5}
- **Visited Edges**: (0-1), (1-2), (2-3), (3-4), (4-5)
- **Unexplored Neighbors**: [] ❌ **NONE - DEAD END**
- **Decision Logic**: 
  - Priority 1 fails (no unexplored neighbors)
  - ✅ **Priority 2**: Find all frontiers (visited nodes with unexplored neighbors)
  - **Frontiers Found**:
    - Node 4: has unexplored neighbors [7, 20]
    - Node 3: has unexplored neighbors [10, 15]
    - Node 2: has unexplored neighbor [6]
    - Node 1: has unexplored neighbor [21]
  - **Nearest Frontier**: Calculated using BFS path length
    - Path from 5→4: length = 1 (closest!)
    - Path from 5→3: length = 2
    - Path from 5→2: length = 3
    - Path from 5→1: length = 4
- **Choice Made**: Node 4 (shortest path = 1 step)
- **Movement**: Backward (B) - returns directly to node 4

---

### **Step 7: At Node 4 (Returned)**
- **Current Node**: 4
- **Unexplored Neighbors**: [7, 20] ⚠️ **TWO CHOICES**
- **Decision Logic**: Back to Priority 1
- **Choice Made**: Node 7 (first option)
- **Movement**: Turn and move forward (depending on heading)

---

### **Step 8: At Node 7**
- **Current Node**: 7 (position: {20, 100})
- **Visited Nodes**: includes 7
- **Visited Edges**: includes (4-7)
- **Unexplored Neighbors**: [8] (only one option)
- **Choice Made**: Node 8
- **Movement**: Forward (F)

---

### **Step 9: At Node 8 - Another Dead End**
- **Current Node**: 8 (position: {20, 120})
- **Unexplored Neighbors**: [] ❌ **NONE**
- **Decision Logic**: Priority 2 - Find nearest frontier
- **Nearest Frontier**: Node 7 (path length = 1)
- **Choice Made**: Node 7
- **Movement**: Backward to node 7

---

### **Step 10: At Node 7 (Returned)**
- **Current Node**: 7
- **Unexplored Neighbors**: [] ❌ **ALL EXPLORED**
- **Decision Logic**: Priority 2 - Find nearest frontier from current position
- **Nearest Frontier**: Node 4 (has unexplored [20])
- **Choice Made**: Move to node 4

---

### **Step 11: At Node 4 (Returned)**
- **Current Node**: 4
- **Unexplored Neighbors**: [20] (one option left)
- **Choice Made**: Node 20
- **Movement**: Forward/move to node 20

---

### **Step 12: At Node 20**
- **Current Node**: 20 (position: {-40, 100})
- **Visited Nodes**: includes 20
- **Visited Edges**: includes (4-20)
- **Unexplored Neighbors**: [] ❌ **DEAD END or all explored**
- **Decision Logic**: Priority 2 - Find nearest frontier
- **Nearest Frontier**: Node 19 (connects to node 20, may have unexplored paths)
- **Choice Made**: Node 19

---

### **Step 13: At Node 19 - TARGET DISCOVERED!**
- **Current Node**: 19 (position: {-40, 80})
- **Visited Nodes**: includes 19
- **Visited Edges**: includes (19-20)
- **Unexplored Neighbors**: [22] ⚠️ **Node 22 is the TARGET!**
- **Decision Logic**: Priority 1 - Explore unexplored neighbor
- **Choice Made**: Node 22 (TARGET NODE!)
- **Target Found Flag**: Set to `true`
- **Movement**: Forward to node 22

---

### **Step 14: At Node 22 - TARGET REACHED, But Exploration Continues**
- **Current Node**: 22 (position: {-60, 80}) ✅ **TARGET NODE!**
- **Visited Nodes**: includes 22
- **Visited Edges**: includes (19-22)
- **Unexplored Neighbors**: [23, 26] ⚠️ **TWO CHOICES**
- **Decision Logic**: Algorithm continues exploring (doesn't stop at target)
- **Choice Made**: Node 23 (first option)
- **Movement**: Forward to node 23

---

### **Step 15: At Node 23**
- **Current Node**: 23 (position: {-60, 120})
- **Visited Nodes**: includes 23
- **Visited Edges**: includes (22-23)
- **Unexplored Neighbors**: [] ❌ **DEAD END**
- **Decision Logic**: Priority 2 - Find nearest frontier
- **Nearest Frontier**: Node 22 (has unexplored [26])
- **Choice Made**: Return to node 22

---

## Key Decision Rules Summary

### **Rule 1: Multiple Unexplored Neighbors**
When a node has multiple unexplored neighbors (like node 1 with options [2, 21]):
- The algorithm takes the **FIRST** neighbor in the adjacency list
- Order depends on how edges were added during graph initialization
- No heuristic (like distance to target) is used - just first-come-first-serve

### **Rule 2: Dead Ends (No Unexplored Neighbors)**
When a node has no unexplored neighbors:
1. Find all **frontiers** (visited nodes that still have unexplored neighbors)
2. Calculate shortest path to each frontier using BFS
3. Choose the **nearest frontier** (shortest path length)
4. Move back along that path

### **Rule 3: All Frontiers Explored**
When no frontiers exist:
- Move back to start node
- Mark exploration as complete

### **Rule 4: Target Found**
When the target is reached:
- Set `target_found_once = true`
- **Continue exploring** (doesn't stop)
- Complete full exploration, then calculate optimal path using Dijkstra's algorithm

---

## Movement Commands Generated

The algorithm generates movement commands:
- **F** = Forward
- **L** = Turn Left (and move forward)
- **R** = Turn Right (and move forward)
- **B** = Backward

The robot tracks its heading (NORTH, EAST, SOUTH, WEST) and calculates turns needed.

---

## After Exploration Complete

Once exploration is done:
1. Calculate **optimal path** from start to target using **Dijkstra's algorithm** (shortest distance, not just steps)
2. Generate movement commands for that optimal path
3. Print both the path and commands

---

## Example: Why Node 2 Instead of Node 21 at Step 2?

At node 1, there were two choices [2, 21]:
- **Node 2**: leads north, part of main path
- **Node 21**: leads west, part of branch path

**The algorithm chose Node 2** simply because:
1. It was the first in the adjacency list (added first in edges_list)
2. The algorithm doesn't use heuristics to choose the "better" path
3. It uses a simple greedy approach: take first available unexplored path

This ensures complete exploration but may not be the most efficient order.

