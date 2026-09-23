/**
 * @file brain.c
 * @brief The SEYED decision core — see inc/brain.h for the contract.
 *
 * This is a thin, deliberate adapter.  Every algorithm it uses (frontier
 * exploration, the time-based early-stop proof, stop-graph Dijkstra for the
 * fast run) already exists in the modules below and already passes its tests.
 * What this file adds is the three things those modules do NOT do:
 *
 *   1. DEAD RECKONING.  The solver takes a coordinate; the robot only supplies
 *      a distance.  brain.c turns one into the other, snapping to the lattice
 *      so encoder drift cannot walk the map off the grid.
 *
 *   2. THE HEADING BRIDGE.  The solver reasons in absolute compass directions;
 *      the robot reports in its own body frame.  The solver already keeps a
 *      heading that matches the robot's facing, so this is a pass-through —
 *      but brain.c is where that is asserted, not assumed.
 *
 *   3. COMMAND GRANULARITY.  The solver emits one command per graph EDGE.  The
 *      robot executes one command per STOP, and those are not the same thing:
 *      the graph contains nodes the robot drives straight through without
 *      stopping.  brain.c reduces a node path to its stop points before turning
 *      it into commands — see _emit_path().
 *
 * The routes themselves are NOT reimplemented here.  Home is
 * maze_graph_shortest_path() and the fast run is maze_fastrun_build_plan(),
 * both already tested; brain.c only converts the node paths they return into
 * the two command strings the caller asked for.  Only the command generation is
 * the brain's own, because only the brain knows which nodes the robot actually
 * stopped at.
 */

#include "brain.h"
#include "maze_solver.h"
#include "maze_explore.h"
#include "maze_graph.h"
#include "maze_robot.h"
#include "maze_fastrun.h"
#include "maze_config.h"

#include <string.h>

/* ==========================================================================
 * STATE
 *
 * Statically allocated — no heap on the target.  One brain per program; the
 * solver has the same constraint (maze_hal.h owns its graph and robot the
 * same way).
 * ========================================================================== */

static MazeGraph s_graph;
static MazeRobot s_robot;

static MazeCoord s_coord;      /**< Brain's dead-reckoned position, cm        */
static uint16_t  s_node;       /**< Node id at the last reported junction     */
static bool      s_exploring;  /**< Still accepting reports                   */

/** Nodes the robot has actually STOPPED at and reported from.
 *
 *  Deliberately the brain's own record, and NOT MazeRobot.visited_nodes.
 *  The solver marks a node visited as soon as it decides to drive there
 *  (maze_robot_move_to), so visited_nodes includes the provisional nodes it
 *  aimed at — exactly the straight passthroughs a plan must skip over.  Reading
 *  that array would reintroduce the very granularity bug _emit_path() exists to
 *  avoid.  This array is only ever written from a real report. */
static bool      s_stopped[MAZE_MAX_NODES];

static uint16_t  s_reports;    /**< Reports accepted                          */
static uint16_t  s_cells;      /**< Cells implied by the last report          */
static uint16_t  s_drift;      /**< |dist − cells×cell| on the last report    */
static uint8_t   s_last_move;  /**< Last move returned, for brain_status()    */

/* The plans.  Sized from MAZE_COMMAND_LOG_SIZE (192), not MAZE_MAX_PATH_LENGTH:
 * a path is a list of NODES but the plan is a list of COMMANDS, and a mission
 * makes up to ~2.6 commands per node (measured: 97 commands on a 37-node maze).
 * Sizing these by path length would silently truncate the plan. */
static char     s_home[MAZE_COMMAND_LOG_SIZE + 1];
static char     s_fast[MAZE_COMMAND_LOG_SIZE + 1];
static uint16_t s_home_len;
static uint16_t s_fast_len;

/* ==========================================================================
 * LATTICE ARITHMETIC
 * ========================================================================== */

/** Unit vector per MazeHeading.  Index order is 0=N, 1=W, 2=S, 3=E — the
 *  firmware's `nav` order, kept deliberately (see STATUS.md design decisions).
 *  Y is up. */
static const int8_t k_dir[4][2] = {
    {  0, +1 },   /* N */
    { -1,  0 },   /* W */
    {  0, -1 },   /* S */
    { +1,  0 },   /* E */
};

/**
 * @brief  Convert a driven distance into whole lattice cells.
 *
 * Rounding to the nearest cell is the point, not an approximation we tolerate.
 * A junction is only ever on a lattice point, so snapping removes encoder error
 * instead of accumulating it — and the early-stop proof, which is geometric,
 * needs node coordinates to actually be on the lattice to bound anything
 * meaningful.  The residual is reported as drift so a real calibration fault is
 * still visible to the operator rather than being silently absorbed forever.
 */
static uint16_t _cells_from_cm(uint16_t cm) {
    uint16_t cells    = (uint16_t)((cm + (MAZE_CELL_CM / 2)) / MAZE_CELL_CM);
    uint16_t snapped  = (uint16_t)(cells * MAZE_CELL_CM);
    s_drift = (cm > snapped) ? (uint16_t)(cm - snapped)
                             : (uint16_t)(snapped - cm);
    return cells;
}

/** The robot's current facing, with the solver's "not set yet" mapped to the
 *  frame's north.  The first report arrives before any move, so this is the
 *  normal path on the first call, not an error case. */
static MazeHeading _heading(void) {
    MazeHeading h = s_robot.heading;
    if ((uint8_t)h > 3) h = MAZE_NORTH;   /* covers MAZE_HEADING_NONE */
    return h;
}

/* ==========================================================================
 * PLAN COLLECTION
 * ========================================================================== */

/**
 * @brief  Turn a node path into the command string the robot can execute.
 *
 * THE SUBTLETY THIS EXISTS FOR.  A command is not "advance one graph edge" — it
 * is "turn as told, then drive to the next place you will stop".  Those are the
 * same thing only when every graph node is a stop point, and they are not.
 *
 * The graph contains nodes the robot never stopped at.  Two sources: the
 * provisional node the solver sprouts at MAZE_CELL_CM when a sensor reports an
 * open branch, which the robot may then drive straight past; and the lattice
 * point a self-healed edge passes through.  Both are provably STRAIGHT
 * passthroughs — if a node had a lateral branch the firmware's side sensors
 * would have fired and the robot would have stopped there — so they are
 * collinear, and dropping them leaves a straight leg of exactly the same shape
 * and length.
 *
 * Get this wrong and the plan silently drifts: the first command drives the
 * robot through several passthroughs to the next real junction, but the next
 * command is still aimed at the node the robot has already passed, so every
 * move after the first is applied to the wrong place.
 *
 * A node counts as a stop point iff the robot actually came to rest there and
 * reported — see `s_stopped`.  NOT `visited_nodes`, which the solver sets for
 * nodes it has merely decided to drive to; those are exactly the provisional
 * nodes created at MAZE_CELL_CM, so reading it would let back in the very nodes
 * this function exists to drop.
 */
static void _emit_path(const uint16_t* path, uint16_t len,
                       char* out, uint16_t* out_len) {
    uint16_t reduced[MAZE_MAX_PATH_LENGTH];
    uint16_t rlen = 0;

    for (uint16_t i = 0; i < len && rlen < MAZE_MAX_PATH_LENGTH; i++) {
        if (i == 0 || s_stopped[path[i]]) reduced[rlen++] = path[i];
    }
    if (rlen < 2) return;                    /* nothing to drive */

    maze_robot_set_path(&s_robot, reduced, rlen);

    for (uint16_t guard = 0; guard <= rlen && maze_robot_has_path(&s_robot); guard++) {
        MazeCommand cmd = maze_robot_step_path(&s_robot);
        if (cmd == MAZE_CMD_NONE) break;
        if (*out_len < MAZE_COMMAND_LOG_SIZE) out[(*out_len)++] = (char)cmd;
    }
}

/**
 * @brief  Finish the mission: produce the homeward route and the fast run.
 *
 * The plans are built here rather than harvested from maze_solver_step()'s own
 * virtual drive, because that drive emits one command per graph edge — the
 * granularity bug described at _emit_path().  The routes themselves still come
 * from the tested modules (maze_graph shortest path, maze_fastrun's stop-graph
 * Dijkstra); only the command generation is the brain's.
 */
static void _finish(void) {
    s_exploring = false;

    /* ---- the way home ---- */
    uint16_t path[MAZE_MAX_PATH_LENGTH];
    uint16_t plen = maze_graph_shortest_path(&s_graph, s_robot.current_node,
                                             s_graph.start_node,
                                             path, MAZE_MAX_PATH_LENGTH);
    if (plen >= 2) _emit_path(path, plen, s_home, &s_home_len);
    s_home[s_home_len] = '\0';

    /* ---- the fast run: start -> target, over the discovered map only ----
     * Emitted after the home route on purpose: _emit_path leaves the robot
     * standing on the start node facing the way the home run ended, which is
     * the pose the fast run's first command assumes.  The two strings therefore
     * concatenate into one executable stream, which is what the caller needs. */
    if (maze_fastrun_build_plan(&s_robot, MAZE_V_MAX_FP, MAZE_ACCEL_FP)) {
        _emit_path(s_robot.fast_path_nodes, s_robot.fast_path_length,
                   s_fast, &s_fast_len);
    }
    s_fast[s_fast_len] = '\0';

    s_robot.phase = MAZE_PHASE_DONE;
    s_last_move   = BRAIN_DONE;
}

/* ==========================================================================
 * API
 * ========================================================================== */

void brain_init(void) {
    memset(&s_graph, 0, sizeof s_graph);
    memset(&s_robot, 0, sizeof s_robot);
    memset(s_stopped, 0, sizeof s_stopped);

    s_coord.x  = 0;
    s_coord.y  = 0;
    s_node     = MAZE_INVALID_NODE;
    s_reports   = 0;
    s_cells     = 0;
    s_drift     = 0;
    s_last_move = BRAIN_DONE;
    s_home_len = 0;
    s_fast_len = 0;
    s_home[0]  = '\0';
    s_fast[0]  = '\0';

    s_exploring = (maze_solver_init(&s_graph, &s_robot, s_coord) == MAZE_OK);

    /* Give the robot a real heading instead of maze_robot_init()'s
     * MAZE_HEADING_NONE.
     *
     * The robot's physical facing at boot IS this frame's north — that is the
     * contract in brain.h, and it is why the caller never has to align the robot
     * with a compass.  Leaving it NONE breaks that, because
     * maze_explore.c:119 derives `is_first` from `heading == NONE`, and
     * maze_robot_compute_command() answers `is_first` with an unconditional
     * FORWARD — it assumes a robot that has not moved yet is already pointing
     * the right way, which in the solver's own virtual drive it is.
     *
     * A real robot is not.  If the start node's only exit is to the side, the
     * correct first move is that turn, and FORWARD drives it into a wall.  The
     * brain's frame is self-consistent and starts pointing north, so the turn
     * can be computed exactly as it would be on any later move.
     *
     * Not fixed inside maze_robot.c: there `is_first` is a deliberate contract
     * for the solver's virtual drive, and the existing tests rely on it. */
    if (s_exploring) s_robot.heading = MAZE_NORTH;
}

uint8_t brain_step(const BrainIn* in) {
    if (!in || !s_exploring) return BRAIN_DONE;

    /* ---- 1. Where is the robot now? -------------------------------------
     * Move it from the previous junction along the facing it left that
     * junction with, by the distance the caller measured.  The heading comes
     * from the solver, which last updated it in maze_robot_move_to() — i.e. it
     * is the direction the robot was TOLD to travel, which is the direction it
     * physically drove. */
    MazeHeading h = _heading();
    uint16_t    n = _cells_from_cm(in->dist_cm);

    s_cells = n;
    s_coord.x = (int16_t)(s_coord.x + (int16_t)(n * MAZE_CELL_CM) * k_dir[h][0]);
    s_coord.y = (int16_t)(s_coord.y + (int16_t)(n * MAZE_CELL_CM) * k_dir[h][1]);

    /* ---- 2. Hand the report to the solver ------------------------------- */
    MazeSensors s;
    s.can_go_forward = (in->front  != 0);
    s.can_go_left    = (in->left   != 0);
    s.can_go_right   = (in->right  != 0);
    s.can_go_back    = (in->back   != 0);
    s.target_reached = (in->target != 0);

    s_node = maze_solver_update_position(&s_robot, s_coord, &s);
    if (s_node == MAZE_INVALID_NODE) {
        /* Graph full or corrupt — stop cleanly rather than emit a move the
         * map cannot justify. */
        _finish();
        return BRAIN_DONE;
    }
    s_reports++;

    /* The robot came to rest HERE and told us so — so this is a place it will
     * stop again, and a plan may end a leg on it.  Recorded from the report
     * rather than inferred from the solver, for the reason given at s_stopped. */
    s_stopped[s_node] = true;

    /* ---- 3. Ask for the next move ---------------------------------------
     *
     * This calls the EXPLORE stage directly rather than maze_solver_step().
     * That is deliberate, and not just because RETURN_HOME/FAST_RUN are stages
     * the brain produces itself:
     *
     * maze_solver_step() signals "exploration is over" by TRANSITIONING — it
     * computes the route home and immediately starts driving it, which calls
     * maze_robot_move_to() and so rewrites `heading`, `current_node`,
     * `current_coord`, and the visited flags.  Everything _finish() needs to
     * read.  Detecting the end from the phase afterwards therefore reads a
     * robot that has already been moved, and the home route comes out planned
     * from the wrong pose — the first turn is silently wrong.
     *
     * maze_explore_step() returns NONE when nothing useful is left and touches
     * nothing, so the decision can be made before any state is destroyed. */
    MazeCommand cmd = maze_explore_step(&s_robot, s_robot.use_proof,
                                        MAZE_V_MAX_FP, MAZE_ACCEL_FP);

    if (cmd == MAZE_CMD_NONE) {
        /* A NONE with a route still pending would be a transit in progress, not
         * the end.  maze_explore_step() cannot return that today — P2 always
         * emits the first leg of a route it plans — but NONE and BRAIN_DONE are
         * both 0, so treating a pending route as "finish" would silently cut
         * the search short.  Check rather than assume. */
        if (maze_robot_has_path(&s_robot)) {
            MazeCommand leg = maze_robot_step_path(&s_robot);
            if (leg != MAZE_CMD_NONE) {
                s_last_move = (uint8_t)leg;
                return s_last_move;
            }
        }
        _finish();
        return BRAIN_DONE;
    }

    s_last_move = (uint8_t)cmd;
    return s_last_move;
}

const char* brain_home_path(void) { return s_home; }
const char* brain_fast_path(void) { return s_fast; }

void brain_status(BrainStatus* out) {
    if (!out) return;
    memset(out, 0, sizeof *out);
    out->move         = s_last_move;
    out->phase        = (uint8_t)s_robot.phase;
    out->node         = s_node;
    out->x            = s_coord.x;
    out->y            = s_coord.y;
    out->reports      = s_reports;
    out->cells        = s_cells;
    out->drift_cm     = s_drift;
    out->target_found = s_robot.target_found ? 1 : 0;
    out->finished     = s_exploring ? 0 : 1;
    out->fast_time_s  = (s_exploring || s_fast_len == 0)
                          ? 0.0f : s_robot.fast_path_time_s;
}
