/**
 * @file maze_utils.c
 * @brief Utility functions implementation
 *
 * This file implements helper functions for coordinate operations,
 * distance calculations, and other utility operations.
 */

#include "maze_utils.h"
#include "maze_graph.h"
#include "maze_config.h"
#include <stdio.h>
#include <math.h>

/*============================================================================
 * COORDINATE UTILITIES
 *============================================================================*/

bool maze_utils_coords_equal(MazeCoord coord1, MazeCoord coord2) {
    int16_t dx = abs(coord1.x - coord2.x);
    int16_t dy = abs(coord1.y - coord2.y);

    return (dx <= MAZE_COORD_TOLERANCE && dy <= MAZE_COORD_TOLERANCE);
}

uint16_t maze_utils_manhattan_distance(MazeCoord coord1, MazeCoord coord2) {
    int16_t dx = abs(coord1.x - coord2.x);
    int16_t dy = abs(coord1.y - coord2.y);

    return (uint16_t)(dx + dy);
}

uint16_t maze_utils_euclidean_distance(MazeCoord coord1, MazeCoord coord2) {
    int16_t dx = coord1.x - coord2.x;
    int16_t dy = coord1.y - coord2.y;

    // Calculate Euclidean distance: sqrt(dx^2 + dy^2)
    float distance = sqrtf((float)(dx * dx + dy * dy));

    return (uint16_t)(distance + 0.5f);  // Round to nearest integer
}

void maze_utils_direction_vector(MazeCoord from, MazeCoord to,
                                  int16_t* dx, int16_t* dy) {
    if (dx == NULL || dy == NULL) {
        return;
    }

    int16_t diff_x = to.x - from.x;
    int16_t diff_y = to.y - from.y;

    // Normalize to -1, 0, or 1
    if (diff_x > 0) {
        *dx = 1;
    } else if (diff_x < 0) {
        *dx = -1;
    } else {
        *dx = 0;
    }

    if (diff_y > 0) {
        *dy = 1;
    } else if (diff_y < 0) {
        *dy = -1;
    } else {
        *dy = 0;
    }
}

/*============================================================================
 * ARRAY UTILITIES
 *============================================================================*/

uint16_t maze_utils_min_uint16(const uint16_t* array, uint16_t length) {
    if (array == NULL || length == 0) {
        return UINT16_MAX;
    }

    uint16_t min_val = array[0];
    for (uint16_t i = 1; i < length; i++) {
        if (array[i] < min_val) {
            min_val = array[i];
        }
    }

    return min_val;
}

uint16_t maze_utils_max_uint16(const uint16_t* array, uint16_t length) {
    if (array == NULL || length == 0) {
        return 0;
    }

    uint16_t max_val = array[0];
    for (uint16_t i = 1; i < length; i++) {
        if (array[i] > max_val) {
            max_val = array[i];
        }
    }

    return max_val;
}

int maze_utils_index_of_min_uint16(const uint16_t* array, uint16_t length) {
    if (array == NULL || length == 0) {
        return -1;
    }

    int min_index = 0;
    uint16_t min_val = array[0];

    for (uint16_t i = 1; i < length; i++) {
        if (array[i] < min_val) {
            min_val = array[i];
            min_index = (int)i;
        }
    }

    return min_index;
}

void maze_utils_reverse_uint16(uint16_t* array, uint16_t length) {
    if (array == NULL || length < 2) {
        return;
    }

    for (uint16_t i = 0; i < length / 2; i++) {
        uint16_t temp = array[i];
        array[i] = array[length - 1 - i];
        array[length - 1 - i] = temp;
    }
}

/*============================================================================
 * VALIDATION UTILITIES
 *============================================================================*/

bool maze_utils_validate_node_id(const MazeGraph* graph, uint16_t node_id) {
    if (graph == NULL) {
        return false;
    }

    return (node_id < graph->node_count);
}

bool maze_utils_validate_coord(MazeCoord coord) {
    // Check if coordinates are within reasonable bounds
    // (adjust based on your maze dimensions)
    const int16_t MAX_COORD = 10000;  // 10 meters in mm
    const int16_t MIN_COORD = -10000;

    return (coord.x >= MIN_COORD && coord.x <= MAX_COORD &&
            coord.y >= MIN_COORD && coord.y <= MAX_COORD);
}

bool maze_utils_validate_sensors(const MazeSensors* sensors) {
    if (sensors == NULL) {
        return false;
    }

    // At least one direction should be available, or we're at target
    return (sensors->can_go_forward ||
            sensors->can_go_left ||
            sensors->can_go_right ||
            sensors->can_go_back ||
            sensors->target_reached);
}

/*============================================================================
 * STRING/DEBUG UTILITIES
 *============================================================================*/

const char* maze_utils_direction_to_string(MazeDirection direction) {
    switch (direction) {
        case MAZE_DIR_STOP:
            return "STOP";
        case MAZE_DIR_FORWARD:
            return "FORWARD";
        case MAZE_DIR_LEFT:
            return "LEFT";
        case MAZE_DIR_RIGHT:
            return "RIGHT";
        case MAZE_DIR_BACK:
            return "BACK";
        default:
            return "UNKNOWN";
    }
}

const char* maze_utils_error_to_string(MazeErrorCode error) {
    switch (error) {
        case MAZE_SUCCESS:
            return "SUCCESS";
        case MAZE_ERR_NULL_PTR:
            return "NULL_PTR";
        case MAZE_ERR_NO_MEMORY:
            return "NO_MEMORY";
        case MAZE_ERR_GRAPH_FULL:
            return "GRAPH_FULL";
        case MAZE_ERR_NODE_NOT_FOUND:
            return "NODE_NOT_FOUND";
        case MAZE_ERR_NO_PATH:
            return "NO_PATH";
        case MAZE_ERR_INVALID_PARAM:
            return "INVALID_PARAM";
        case MAZE_ERR_NOT_INITIALIZED:
            return "NOT_INITIALIZED";
        default:
            return "UNKNOWN_ERROR";
    }
}

#if MAZE_DEBUG_ENABLED

void maze_utils_print_coord(MazeCoord coord) {
    MAZE_DEBUG_PRINT("(%d, %d)", coord.x, coord.y);
}

void maze_utils_print_node(const MazeNode* node) {
    if (node == NULL) {
        MAZE_DEBUG_PRINT("Node: NULL");
        return;
    }

    MAZE_DEBUG_PRINT("Node %d at (%d, %d) %s",
                     node->id,
                     node->coord.x,
                     node->coord.y,
                     node->visited ? "[VISITED]" : "");
}

void maze_utils_print_direction(MazeDirection direction) {
    MAZE_DEBUG_PRINT("%s", maze_utils_direction_to_string(direction));
}

#endif // MAZE_DEBUG_ENABLED

/*============================================================================
 * MATH UTILITIES
 *============================================================================*/

int16_t maze_utils_abs_int16(int16_t value) {
    return (value < 0) ? -value : value;
}

int16_t maze_utils_min_int16(int16_t a, int16_t b) {
    return (a < b) ? a : b;
}

int16_t maze_utils_max_int16(int16_t a, int16_t b) {
    return (a > b) ? a : b;
}

int16_t maze_utils_clamp_int16(int16_t value, int16_t min, int16_t max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}
