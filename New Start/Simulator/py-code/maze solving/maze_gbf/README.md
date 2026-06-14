# Maze GBF Navigation System

**Greedy Best-First Frontier-Based Exploration for STM32 Microcontrollers**

This is a C implementation of the V12 Greedy Best-First maze exploration algorithm, converted from Python (`sim_V12_GBF_stable.py`) for embedded systems deployment on STM32 microcontrollers.

## Overview

The navigation system autonomously explores mazes using a 3-priority decision system:

1. **Priority 1**: Explore unexplored paths at current location
2. **Priority 2**: Navigate to nearest frontier (boundary of known space)
3. **Priority 3**: Return to start when exploration is complete

### Key Features

- ✅ Complete graph-based maze exploration
- ✅ Frontier-based exploration (optimal path ordering)
- ✅ Memory efficient (~1.5KB for 50-node maze)
- ✅ No dynamic memory allocation during operation
- ✅ Suitable for STM32 with >8KB RAM
- ✅ Comprehensive test suite
- ✅ Easy hardware integration

## Hardware Requirements

- **Microcontroller**: STM32 (or any ARM Cortex-M with >8KB RAM)
- **Sensors**: IR line sensors for intersection detection
- **Motors**: 2-wheel differential drive or similar
- **Localization**: Coordinate system (odometry, sensors, etc.)

## Quick Start

### 1. Build and Test on Host

```bash
cd maze_gbf
make test
```

This compiles the library and runs all unit and integration tests.

### 2. Integration Steps

```c
#include "maze_gbf.h"

// Initialize system
MazeRobot robot;
MazeGraph graph;
maze_gbf_init(&robot, &graph, 0, 0);  // Start at (0, 0)

// Main navigation loop
while (!maze_gbf_is_exploration_complete(&robot)) {
    // Read sensors
    MazeSensors sensors;
    sensors.can_go_forward = check_forward_sensor();
    sensors.can_go_left = check_left_sensor();
    sensors.can_go_right = check_right_sensor();
    sensors.can_go_back = check_back_sensor();
    sensors.target_reached = check_all_black();

    // Get current position from localization
    int16_t x = get_current_x();
    int16_t y = get_current_y();

    // Get next move from navigation algorithm
    MazeDirection dir = maze_gbf_get_next_move(&robot, &sensors, x, y);

    // Execute movement
    switch (dir) {
        case MAZE_DIR_FORWARD:
            move_forward();
            break;
        case MAZE_DIR_LEFT:
            turn_left();
            move_forward();
            break;
        case MAZE_DIR_RIGHT:
            turn_right();
            move_forward();
            break;
        case MAZE_DIR_BACK:
            move_backward();
            break;
        case MAZE_DIR_STOP:
            // Exploration complete
            break;
    }
}

// Clean up
maze_gbf_cleanup(&robot, &graph);
```

## API Reference

### Core Functions

#### `maze_gbf_init()`
```c
int maze_gbf_init(MazeRobot* robot, MazeGraph* graph,
                  int16_t start_x, int16_t start_y);
```
Initialize the navigation system at starting coordinates.

**Returns**: `MAZE_SUCCESS` on success, error code on failure

#### `maze_gbf_get_next_move()`
```c
MazeDirection maze_gbf_get_next_move(MazeRobot* robot,
                                     const MazeSensors* sensors,
                                     int16_t current_x,
                                     int16_t current_y);
```
Get the next movement direction based on current state and sensors.

**Returns**: Direction to move (`MAZE_DIR_FORWARD`, `MAZE_DIR_LEFT`, `MAZE_DIR_RIGHT`, `MAZE_DIR_BACK`, or `MAZE_DIR_STOP`)

#### `maze_gbf_cleanup()`
```c
void maze_gbf_cleanup(MazeRobot* robot, MazeGraph* graph);
```
Free all resources used by the navigation system.

### Data Structures

#### `MazeSensors`
```c
typedef struct {
    bool can_go_forward;   // Forward path available
    bool can_go_left;      // Left turn available
    bool can_go_right;     // Right turn available
    bool can_go_back;      // Backward path available
    bool target_reached;   // All sensors detect black (target)
} MazeSensors;
```

#### `MazeDirection`
```c
typedef enum {
    MAZE_DIR_STOP,         // Stop movement
    MAZE_DIR_FORWARD,      // Move forward
    MAZE_DIR_LEFT,         // Turn left
    MAZE_DIR_RIGHT,        // Turn right
    MAZE_DIR_BACK          // Move backward
} MazeDirection;
```

## Configuration

Edit `maze_config.h` to adjust parameters:

```c
#define MAZE_MAX_NODES          50    // Maximum nodes in maze
#define MAZE_MAX_EDGES          100   // Maximum edges
#define MAZE_COORD_TOLERANCE    2     // Coordinate matching tolerance (mm)
#define MAZE_EXPLORATION_TIMEOUT 500  // Maximum exploration steps
#define MAZE_DEBUG_ENABLED      1     // Enable debug output
```

## Memory Usage

For `MAZE_MAX_NODES=50` and `MAZE_MAX_EDGES=100`:

- **Static memory**: ~1.4 KB
- **Temporary buffers**: ~100 bytes
- **Total**: ~1.5 KB (18.75% of 8KB)

## Building for STM32

### Using ARM GCC Toolchain

```bash
# Compile library
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 \
    -I./inc \
    -c src/maze_gbf.c -o build/maze_gbf.o
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 \
    -I./inc \
    -c src/maze_graph.c -o build/maze_graph.o
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 \
    -I./inc \
    -c src/maze_robot.c -o build/maze_robot.o
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 \
    -I./inc \
    -c src/maze_utils.c -o build/maze_utils.o

# Archive library
arm-none-eabi-ar rcs libmaze_gbf.a build/*.o
```

### STM32CubeIDE Integration

1. Copy `inc/` directory to your project's include path
2. Copy `src/` files to your project's source directory
3. Add `#include "maze_gbf.h"` to your main code
4. Call API functions as shown in Quick Start

## Testing

### Run All Tests
```bash
make test
```

### Run Unit Tests Only
```bash
make test-unit
```

### Run Integration Tests Only
```bash
make test-integration
```

### Clean Build
```bash
make clean
make test
```

## Hardware Abstraction Layer

You need to implement these functions for your hardware:

```c
// Sensor reading
bool check_forward_sensor(void);
bool check_left_sensor(void);
bool check_right_sensor(void);
bool check_back_sensor(void);
bool check_all_black(void);  // Target detection

// Localization
int16_t get_current_x(void);  // Get X coordinate
int16_t get_current_y(void);  // Get Y coordinate

// Motor control
void move_forward(void);
void turn_left(void);
void turn_right(void);
void move_backward(void);
void stop_motors(void);
```

## Algorithm Details

### Frontier-Based Exploration

The algorithm maintains a "known map" of explored nodes and edges. At each step:

1. **Local Exploration**: If current node has unexplored paths, take the first one
2. **Frontier Navigation**: Find the nearest "frontier" (visited node with unexplored neighbors) and navigate to it
3. **Return Home**: When no frontiers remain, return to start

### Graph Building

As the robot explores:
- New nodes are created when reaching new coordinates (within tolerance)
- Edges are added when moving between nodes
- Edges are marked "explored" when traversed

### Coordinate System

- Units: Typically millimeters (configurable)
- Origin: Start position (0, 0)
- Tolerance: 2mm default for coordinate matching

## File Structure

```
maze_gbf/
├── inc/
│   ├── maze_gbf.h           # Main API
│   ├── maze_types.h         # Type definitions
│   ├── maze_graph.h         # Graph operations
│   ├── maze_robot.h         # Robot state management
│   ├── maze_utils.h         # Utility functions
│   └── maze_config.h        # Configuration
├── src/
│   ├── maze_gbf.c           # Main algorithm
│   ├── maze_graph.c         # Graph implementation
│   ├── maze_robot.c         # Robot operations
│   └── maze_utils.c         # Utilities
├── test/
│   ├── test_maze_gbf.c      # Unit tests
│   └── integration_test.c   # Integration tests
├── Makefile                 # Build system
└── README.md                # This file
```

## Performance Characteristics

- **Time Complexity**: O(V + E) where V = nodes, E = edges
- **Space Complexity**: O(V + E) for graph storage
- **Exploration Completeness**: Guaranteed (explores all reachable nodes)
- **Path Optimality**: Computes shortest path after exploration

## Troubleshooting

### Problem: Robot gets stuck in loops
**Solution**: Check coordinate tolerance in `maze_config.h`. Increase if localization is noisy.

### Problem: Memory allocation fails
**Solution**: Reduce `MAZE_MAX_NODES` and `MAZE_MAX_EDGES` in `maze_config.h`.

### Problem: Wrong direction decisions
**Solution**: Verify sensor inputs are correct. Check that `can_go_*` flags match actual maze geometry.

### Problem: Exploration never completes
**Solution**: Check for unreachable areas in maze. Verify robot can physically access all regions.

## Contributing

This is a conversion of the Python implementation in `sim_V12_GBF_stable.py`. For algorithm improvements, start with the Python version first.

## License

Same as parent project (see main repository license).

## Authors

- Original Python implementation: See `sim_V12_GBF_stable.py`
- C conversion and embedded optimization: 2024

## Version History

- **v1.0.0** (2024-04-24): Initial C conversion from Python V12
  - Complete graph operations
  - Frontier-based exploration
  - STM32 optimization
  - Comprehensive test suite
