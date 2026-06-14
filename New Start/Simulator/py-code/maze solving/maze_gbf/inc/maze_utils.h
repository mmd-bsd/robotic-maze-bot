#ifndef MAZE_UTILS_H
#define MAZE_UTILS_H

#include "maze_types.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @file maze_utils.h
 * @brief Utility functions for maze navigation
 *
 * This file declares helper functions for coordinate operations,
 * distance calculations, and other utility operations.
 */

/*============================================================================
 * COORDINATE UTILITIES
 *============================================================================*/

/**
 * @brief Compare two coordinates for equality
 *
 * Checks if two coordinates are within MAZE_COORD_TOLERANCE distance.
 *
 * @param coord1 First coordinate
 * @param coord2 Second coordinate
 * @return true if coordinates are equal (within tolerance), false otherwise
 */
bool maze_utils_coords_equal(MazeCoord coord1, MazeCoord coord2);

/**
 * @brief Calculate Manhattan distance between two coordinates
 *
 * Returns the Manhattan distance (|dx| + |dy|) between two points.
 * Useful for heuristic calculations.
 *
 * @param coord1 First coordinate
 * @param coord2 Second coordinate
 * @return Manhattan distance
 */
uint16_t maze_utils_manhattan_distance(MazeCoord coord1, MazeCoord coord2);

/**
 * @brief Calculate Euclidean distance between two coordinates
 *
 * Returns the Euclidean distance (sqrt(dx^2 + dy^2)) between two points.
 * More accurate but slower than Manhattan distance.
 *
 * @param coord1 First coordinate
 * @param coord2 Second coordinate
 * @return Euclidean distance (rounded to nearest integer)
 */
uint16_t maze_utils_euclidean_distance(MazeCoord coord1, MazeCoord coord2);

/**
 * @brief Calculate direction vector between two coordinates
 *
 * Calculates the normalized direction vector from one point to another.
 * Useful for smooth movement calculations.
 *
 * @param from Starting coordinate
 * @param to Target coordinate
 * @param dx Output: X component of direction vector (-1, 0, or 1)
 * @param dy Output: Y component of direction vector (-1, 0, or 1)
 */
void maze_utils_direction_vector(MazeCoord from, MazeCoord to,
                                  int16_t* dx, int16_t* dy);

/*============================================================================
 * ARRAY UTILITIES
 *============================================================================*/

/**
 * @brief Find minimum value in array
 *
 * Returns the minimum value in an array of uint16_t.
 *
 * @param array Input array
 * @param length Array length
 * @return Minimum value, or UINT16_MAX if array is empty
 */
uint16_t maze_utils_min_uint16(const uint16_t* array, uint16_t length);

/**
 * @brief Find maximum value in array
 *
 * Returns the maximum value in an array of uint16_t.
 *
 * @param array Input array
 * @param length Array length
 * @return Maximum value, or 0 if array is empty
 */
uint16_t maze_utils_max_uint16(const uint16_t* array, uint16_t length);

/**
 * @brief Find index of minimum value in array
 *
 * Returns the index of the minimum value in an array.
 *
 * @param array Input array
 * @param length Array length
 * @return Index of minimum value, or -1 if array is empty
 */
int maze_utils_index_of_min_uint16(const uint16_t* array, uint16_t length);

/**
 * @brief Reverse array in place
 *
 * Reverses the order of elements in an array.
 *
 * @param array Array to reverse
 * @param length Array length
 */
void maze_utils_reverse_uint16(uint16_t* array, uint16_t length);

/*============================================================================
 * VALIDATION UTILITIES
 *============================================================================*/

/**
 * @brief Validate node ID
 *
 * Checks if a node ID is valid (within range and exists in graph).
 *
 * @param graph Pointer to graph structure
 * @param node_id Node ID to validate
 * @return true if valid, false otherwise
 */
bool maze_utils_validate_node_id(const MazeGraph* graph, uint16_t node_id);

/**
 * @brief Validate coordinates
 *
 * Checks if coordinates are within reasonable bounds.
 * (Not negative and within typical maze dimensions).
 *
 * @param coord Coordinates to validate
 * @return true if valid, false otherwise
 */
bool maze_utils_validate_coord(MazeCoord coord);

/**
 * @brief Validate sensor data
 *
 * Checks if sensor data is valid (at least one direction available,
 * or at target).
 *
 * @param sensors Pointer to sensor structure
 * @return true if valid, false otherwise
 */
bool maze_utils_validate_sensors(const MazeSensors* sensors);

/*============================================================================
 * STRING/DEBUG UTILITIES
 *============================================================================*/

/**
 * @brief Convert direction to string
 *
 * Returns a human-readable string representation of a direction.
 *
 * @param direction Direction to convert
 * @return String representation ("STOP", "FORWARD", "LEFT", "RIGHT", "BACK")
 */
const char* maze_utils_direction_to_string(MazeDirection direction);

/**
 * @brief Convert error code to string
 *
 * Returns a human-readable string representation of an error code.
 *
 * @param error Error code to convert
 * @return String representation
 */
const char* maze_utils_error_to_string(MazeErrorCode error);

#if MAZE_DEBUG_ENABLED

/**
 * @brief Print coordinate to debug output
 *
 * Prints coordinates in format "(x, y)".
 *
 * @param coord Coordinate to print
 */
void maze_utils_print_coord(MazeCoord coord);

/**
 * @brief Print node info to debug output
 *
 * Prints node ID and coordinates.
 *
 * @param node Pointer to node structure
 */
void maze_utils_print_node(const MazeNode* node);

/**
 * @brief Print direction to debug output
 *
 * Prints human-readable direction.
 *
 * @param direction Direction to print
 */
void maze_utils_print_direction(MazeDirection direction);

#endif // MAZE_DEBUG_ENABLED

/*============================================================================
 * MATH UTILITIES
 *============================================================================*/

/**
 * @brief Absolute value for int16_t
 *
 * Returns the absolute value of a signed 16-bit integer.
 *
 * @param value Input value
 * @return Absolute value
 */
int16_t maze_utils_abs_int16(int16_t value);

/**
 * @brief Minimum of two int16_t values
 *
 * Returns the smaller of two values.
 *
 * @param a First value
 * @param b Second value
 * @return Minimum value
 */
int16_t maze_utils_min_int16(int16_t a, int16_t b);

/**
 * @brief Maximum of two int16_t values
 *
 * Returns the larger of two values.
 *
 * @param a First value
 * @param b Second value
 * @return Maximum value
 */
int16_t maze_utils_max_int16(int16_t a, int16_t b);

/**
 * @brief Clamp value to range
 *
 * Clamps a value to be within the specified range [min, max].
 *
 * @param value Value to clamp
 * @param min Minimum value
 * @param max Maximum value
 * @return Clamped value
 */
int16_t maze_utils_clamp_int16(int16_t value, int16_t min, int16_t max);

#endif // MAZE_UTILS_H
