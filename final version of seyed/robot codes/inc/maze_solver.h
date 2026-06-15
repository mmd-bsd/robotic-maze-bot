/**
 * @file maze_solver.h
 * @brief Top-level mission state machine: EXPLORE → RETURN_HOME → FAST_RUN → DONE.
 *
 * Wires together the graph, robot, exploration, proof, and fast-run modules.
 *
 * Usage:
 *   MazeGraph g;
 *   MazeRobot r;
 *   maze_solver_init(&g, &r, start_coord);
 *   while (r.phase != MAZE_PHASE_DONE) {
 *       MazeCommand cmd = maze_solver_step(&r, v_max_fp, a_max_fp);
 *       // execute `cmd` on the real robot (via maze_cmd_to_cross → firmware)
 *   }
 *
 * All units are real-world (1 unit = 1 cm).  Motion parameters use fixed-point
 * ×100 (e.g., 100.00 cm/s → 10000).
 */

#ifndef MAZE_SOLVER_H
#define MAZE_SOLVER_H

#include "maze_types.h"
#include "maze_robot.h"
#include "maze_graph.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialise the solver: create the start node and robot.
 * @return MAZE_OK on success.
 */
int maze_solver_init(MazeGraph* graph, MazeRobot* robot, MazeCoord start);

/**
 * @brief  Execute ONE step of the mission state machine.
 *
 * Automatically transitions phases:
 *   EXPLORE → RETURN_HOME  (when no useful frontiers remain)
 *   RETURN_HOME → FAST_RUN (when robot reaches the start node)
 *   FAST_RUN → DONE        (when robot reaches the target)
 *
 * @param  robot     Robot state.
 * @param  v_max_fp  Max speed (cm/s × 100).
 * @param  a_max_fp  Max acceleration (cm/s² × 100).
 * @return The command (F/L/R/B) to execute, or MAZE_CMD_NONE if done.
 */
MazeCommand maze_solver_step(MazeRobot* robot,
                             int32_t v_max_fp,
                             int32_t a_max_fp);

/**
 * @brief  Notify the solver that the robot has reached a sensor reading
 *         at a known coordinate.  The solver will create or match a node
 *         and update the graph.
 *
 * @param  robot   Robot state.
 * @param  coord   Current coordinates (from odometry / encoder tracking).
 * @param  sensors Sensor readings at this position.
 * @return Node index at this position.
 */
uint16_t maze_solver_update_position(MazeRobot* robot,
                                     MazeCoord coord,
                                     const MazeSensors* sensors);

#ifdef __cplusplus
}
#endif

#endif /* MAZE_SOLVER_H */
