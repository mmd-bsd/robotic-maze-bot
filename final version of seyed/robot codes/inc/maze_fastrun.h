/**
 * @file maze_fastrun.h
 * @brief Acceleration-aware fast run and time-optimal path (ALGORITHMS.md §6).
 *
 * The fast run uses a trapezoidal velocity profile: the robot accelerates
 * on clear straights and stops (v=0) at every turn.  The fastest path therefore
 * minimises TIME, not distance — a longer straighter route can beat a shorter
 * twisty one.
 *
 * Key functions:
 *   run_time(L)         — drive time for a straight of length L (centiseconds)
 *   speed_at(s, L)      — speed at arc-distance s into a run of length L
 *   time_optimal_path() — Dijkstra on stop-graph (minimum-TIME route)
 *   fast_run_plan()     — precompute trajectory for smooth animation / execution
 */

#ifndef MAZE_FASTRUN_H
#define MAZE_FASTRUN_H

#include "maze_types.h"
#include "maze_robot.h"
#include "maze_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Compute the drive time for a straight run of length `L_cm` under
 *         the trapezoidal velocity profile (starts and ends at v=0).
 *
 * @param  L_cm        Length of the straight run (cm).
 * @param  v_max_fp    Fixed-point max speed (cm/s × 100).
 * @param  a_max_fp    Fixed-point max acceleration (cm/s² × 100).
 * @return Drive time in centiseconds (seconds × 100).
 */
uint32_t maze_run_time_cs(float L_cm, int32_t v_max_fp, int32_t a_max_fp);

/**
 * @brief  Compute the instantaneous speed at arc-distance `s` into a run
 *         of length `L_cm` that starts and ends at v=0.
 *
 * @return Speed in cm/s.
 */
float maze_speed_at_cm(float s, float L_cm, int32_t v_max_fp, int32_t a_max_fp);

/**
 * @brief  Find the minimum-TIME path from start to target on the KNOWN MAP
 *         using the stop-graph Dijkstra algorithm.
 *
 *         Because the robot stops at every turn, the search state is just
 *         the node.  From each node, we enumerate maximal straight runs in
 *         each cardinal direction; each reached node is a candidate with
 *         cost = run_time(total_distance).
 *
 * @param  graph       The known map (only explored edges are traversed).
 * @param  start       Start node index.
 * @param  target      Target node index.
 * @param  v_max_fp    Fixed-point max speed.
 * @param  a_max_fp    Fixed-point max acceleration.
 * @param  path_out    Output: node sequence of the fastest path.
 * @param  max_len     Capacity of `path_out`.
 * @return Path length (nodes), or 0 if no path exists.
 */
uint16_t maze_time_optimal_path(const MazeGraph* graph,
                                uint16_t start,
                                uint16_t target,
                                int32_t v_max_fp,
                                int32_t a_max_fp,
                                uint16_t* path_out,
                                uint16_t max_len);

/**
 * @brief  Return the total drive TIME (centiseconds) of the time-optimal
 *         path from start to target.
 */
uint32_t maze_time_optimal_cost(const MazeGraph* graph,
                                uint16_t start,
                                uint16_t target,
                                int32_t v_max_fp,
                                int32_t a_max_fp);

/**
 * @brief  Precompute a trajectory for the fast run (sequence of t→(pos,speed)
 *         points) for smooth execution on the robot.
 *
 * @param  robot   Robot state — fills robot->fast_traj and related fields.
 * @param  v_max_fp, a_max_fp  Motion parameters.
 * @return true on success, false if no valid path exists.
 */
bool maze_fastrun_build_plan(MazeRobot* robot,
                             int32_t v_max_fp,
                             int32_t a_max_fp);

#ifdef __cplusplus
}
#endif

#endif /* MAZE_FASTRUN_H */
