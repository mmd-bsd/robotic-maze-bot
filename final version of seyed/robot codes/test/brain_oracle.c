/*============================================================================
 * brain_oracle.c -- the decision core as a stdin/stdout filter.
 *
 * WHAT THIS IS
 * `brain.c` is a pure decision function: BrainIn in, one move out.  That makes
 * it drivable from outside the firmware, and this is the driver.  Feed it the
 * junctions a robot reported and it answers with the moves the brain should
 * have made -- using THE SAME CODE THE ROBOT RUNS, not a re-implementation.
 *
 * That distinction is the whole point.  A second implementation of the same
 * algorithm can disagree with the robot and leave you unable to say which side
 * is wrong.  Here, any disagreement is a finding about the robot or the wire.
 * What it CANNOT do is prove the brain's model of the robot is right -- only
 * the physical robot can, which is why this is a bring-up instrument and not a
 * substitute for one.
 *
 * IT NEEDS NO MAZE.  Unlike brain_host.c (which drives the brain from a *model*
 * of a robot and so needs a maze to model), this reads the robot's own reports
 * and lets the brain discover the map from them.  So it builds and RUNS in the
 * normal test suite, where brain.c was previously compile-only.
 *
 * PROTOCOL
 *   stdin, one command per line, '#' starts a comment:
 *     INIT                             -> brain_init()      -> "OK init"
 *     <L> <R> <F> <B> <TARGET> <DIST>  -> brain_step()      -> "S <fields...>"
 *     QUIT                             -> exit 0
 *   stdout:
 *     OK init
 *     S <move> <phase> <node> <x> <y> <reports> <cells> <drift>
 *       <target_found> <finished> <fast_time_s>
 *     HOME <string>            (only once finished)
 *     FAST <string>            (only once finished)
 *     E <reason>               (bad input -- never silent)
 *
 *   <move> is F/L/R/B, or 'D' when brain_step() returned BRAIN_DONE.
 *
 * TWO THINGS THAT BITE
 *   1. stdout is a pipe here, so it is fully buffered by default and the
 *      reader waits for a flush that never comes. Line-buffer it (below).
 *   2. brain.c's state is static. One process is one mission: a second replay
 *      needs a fresh process, or an INIT.
 *
 * Usage:
 *   build/brain_oracle.exe                 # filter mode (what the app uses)
 *   build/brain_oracle.exe --probe         # print the selftest moves, unchecked
 *   build/brain_oracle.exe --selftest      # check them; exit 1 on any mismatch
 *============================================================================*/

#include <stdio.h>
#include <string.h>

#include "brain.h"

/* ------------------------------------------------------------------ status */

static void emit_status(unsigned char move)
{
    BrainStatus st;

    brain_status(&st);

    printf("S %c %u %u %d %d %u %u %u %u %u %.4f\n",
           (move == (unsigned char)BRAIN_DONE) ? 'D' : (char)move,
           (unsigned)st.phase, (unsigned)st.node,
           (int)st.x, (int)st.y,
           (unsigned)st.reports, (unsigned)st.cells, (unsigned)st.drift_cm,
           (unsigned)st.target_found, (unsigned)st.finished,
           (double)st.fast_time_s);

    if (st.finished)
    {
        printf("HOME %s\n", brain_home_path());
        printf("FAST %s\n", brain_fast_path());
    }

    /* Explicit, and NOT left to setvbuf.  MinGW does not honour _IOLBF on a
       pipe, so without this every response sits in the C library's buffer until
       the process exits -- which is invisible when a shell pipes a whole file
       in, and a deadlock when a host process reads one answer at a time. */
    fflush(stdout);
}

/* ==========================================================================
 * SELF TEST
 *
 * The sequence is a 2x2 square driven from the start node, with the target
 * sitting on the third corner:
 *
 *      B ---- C           A (0,0)  start, robot faces frame-north
 *      |      |           B (0,20) one lateral exit (east)
 *      A ---- D           C (20,20) THE TARGET DISC
 *                         D (20,0) one lateral exit (west)
 *
 * Each entry is one junction the robot stops at, as the firmware's
 * brain_report() would have built it.  `want` is the move the brain must
 * answer with; run `--probe` to print the sequence, and if the brain's
 * behaviour is deliberately changed, update `want` FROM that output -- do not
 * guess.  The moves below were taken from a --probe run on 2026-09-23.
 * ========================================================================== */

typedef struct {
    int L, R, F, B, T;
    unsigned dist;
    char want;
} OracleStep;

static const OracleStep STEPS[] = {
    /* at A, facing north: the only exit is straight ahead into the corridor */
    { 0, 0, 1, 1, 0,   0,  'F' },
    /* at B, facing north: nothing ahead, one lateral to the right */
    { 0, 1, 0, 1, 0,  20,  'R' },
    /* at C, facing east: nothing ahead, one lateral to the right -- TARGET */
    { 0, 1, 0, 1, 1,  20,  'R' },
    /* at D, facing south: nothing ahead, one lateral to the right */
    { 0, 1, 0, 1, 0,  20,  'R' },
    /* back at A, facing west: nothing ahead, no lateral -- the square is shut.
       Everything is explored and the target was seen, so exploration must end. */
    { 0, 0, 0, 1, 0,  20,  'D' },
};

#define N_STEPS ((int)(sizeof STEPS / sizeof STEPS[0]))

static int selftest(int check)
{
    int i, fails = 0;

    brain_init();

    for (i = 0; i < N_STEPS; i++)
    {
        BrainIn in;
        unsigned char move;
        char got;

        in.left   = (unsigned char)STEPS[i].L;
        in.right  = (unsigned char)STEPS[i].R;
        in.front  = (unsigned char)STEPS[i].F;
        in.back   = (unsigned char)STEPS[i].B;
        in.target = (unsigned char)STEPS[i].T;
        in.dist_cm = (unsigned short)STEPS[i].dist;

        move = brain_step(&in);
        got  = (move == (unsigned char)BRAIN_DONE) ? 'D' : (char)move;

        printf("  step %d: L%d R%d F%d B%d T%d dist=%-3u -> %c",
               i, STEPS[i].L, STEPS[i].R, STEPS[i].F, STEPS[i].B,
               STEPS[i].T, STEPS[i].dist, got);

        /* Invariants that hold whatever the algorithm does.  A move outside
           the alphabet would drive the robot through replay_dispatch()'s
           switch and off the end of it. */
        if (!strchr("FLRBD", got))
        {
            printf("   << NOT A MOVE");
            fails++;
        }
        if (check && got != STEPS[i].want)
        {
            printf("   << WANT %c", STEPS[i].want);
            fails++;
        }
        printf("\n");
    }

    /* The end state, checked independently of the moves that produced it. */
    {
        BrainStatus st;
        brain_status(&st);

        printf("  final: reports=%u cells=%u drift=%u target_found=%u"
               " finished=%u fast_time=%.4f s\n",
               (unsigned)st.reports, (unsigned)st.cells, (unsigned)st.drift_cm,
               (unsigned)st.target_found, (unsigned)st.finished,
               (double)st.fast_time_s);
        printf("  home=\"%s\"  fast=\"%s\"\n",
               brain_home_path(), brain_fast_path());

        if (!st.target_found) { printf("  << TARGET NEVER FOUND\n");     fails++; }
        if (!st.finished)     { printf("  << NEVER FINISHED\n");         fails++; }
        if (st.reports != (unsigned)N_STEPS)
        { printf("  << report count != steps fed\n");                    fails++; }
        if (st.drift_cm != 0)
        { printf("  << every link was a whole cell, so drift must be 0\n"); fails++; }

        /* The two plans share one alphabet with replay_dispatch().  A 'B' is
           expected here (the square forces a reversal), and the legacy replay
           stages had no 'B' case until 2026-09-23. */
        {
            const char* p;
            for (p = brain_home_path(); *p; p++)
                if (!strchr("FLRB", *p)) { printf("  << HOME has %c\n", *p); fails++; }
            for (p = brain_fast_path(); *p; p++)
                if (!strchr("FLRB", *p)) { printf("  << FAST has %c\n", *p); fails++; }
        }
    }

    if (check)
        printf("brain_oracle selftest: %s (%d steps, %d failure(s))\n",
               fails ? "FAIL" : "PASS", N_STEPS, fails);

    return fails ? 1 : 0;
}

/* -------------------------------------------------------------------- main */

int main(int argc, char** argv)
{
    char line[256];

    /* The pipe: without this the app blocks waiting for output that is sitting
       in the C library's buffer.  Line buffering also means one request is
       answered by exactly one flush, so a live run never lags. */
    setvbuf(stdout, NULL, _IOLBF, 0);
    setvbuf(stdin,  NULL, _IOLBF, 0);

    if (argc > 1 && strcmp(argv[1], "--probe") == 0)
    {
        printf("brain_oracle --probe (no expectations, printing what it does)\n");
        return selftest(0);
    }
    if (argc > 1 && strcmp(argv[1], "--selftest") == 0)
    {
        return selftest(1);
    }

    brain_init();

    /* No banner: the protocol is strictly one reply per request, so a host that
       writes INIT and reads once cannot pick up a stray queued line and slip a
       junction behind.  Silence until asked is the whole point. */
    while (fgets(line, sizeof line, stdin) != NULL)
    {
        int l, r, f, b, t;
        unsigned long d;

        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
            continue;
        if (strncmp(line, "INIT", 4) == 0)
        {
            brain_init();
            printf("OK init\n");
            fflush(stdout);
            continue;
        }
        if (strncmp(line, "QUIT", 4) == 0)
            break;

        if (sscanf(line, "%d %d %d %d %d %lu", &l, &r, &f, &b, &t, &d) == 6)
        {
            BrainIn in;
            in.left   = (unsigned char)(l ? 1 : 0);
            in.right  = (unsigned char)(r ? 1 : 0);
            in.front  = (unsigned char)(f ? 1 : 0);
            in.back   = (unsigned char)(b ? 1 : 0);
            in.target = (unsigned char)(t ? 1 : 0);
            in.dist_cm = (unsigned short)d;
            emit_status(brain_step(&in));
            continue;
        }

        /* Anything else is echoed as an error rather than ignored -- a lost
           junction shows up as a desynchronised stream, and silence would
           make that look like agreement. */
        printf("E bad input\n");
        fflush(stdout);
    }

    return 0;
}
