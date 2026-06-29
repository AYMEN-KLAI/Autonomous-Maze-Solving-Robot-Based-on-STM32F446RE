/*
/* ============================================================
 * maze.h  —  Flood-Fill Maze Solver
 * STM32F446RE · 7×7 grid · 40×40 cm cells
 * ============================================================ */
#ifndef MAZE_H
#define MAZE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================
   MAZE GEOMETRY
   ============================================================ */
#define MAZE_W          7           /* columns (X) */
#define MAZE_H          7           /* rows    (Y) */
#define CELL_MM         400.0f      /* cell side in mm */

/* Goal: centre 2×2 block  (col 3, row 3) */
#define GOAL_C0         3
#define GOAL_R0         3

/* ============================================================
   WALL BITMASK  (stored per cell — which walls THAT cell has)
   ============================================================ */
#define WALL_N  0x01    /* North (+Y) */
#define WALL_E  0x02    /* East  (+X) */
#define WALL_S  0x04    /* South (-Y) */
#define WALL_W  0x08    /* West  (-X) */
#define WALL_VISITED 0x10   /* cell visited flag */

/* ============================================================
   ROBOT HEADING  (cardinal, 0-based, CW)
   ============================================================ */
typedef enum {
    DIR_N = 0,
    DIR_E = 1,
    DIR_S = 2,
    DIR_W = 3
} Direction;

/* ============================================================
   PUBLIC API
   ============================================================ */

/**
 * maze_init()
 *   Build border walls, reset flood-fill distances, mark start.
 *   Call once after all peripherals are initialised.
 */
void maze_init(void);

/**
 * maze_run()
 *   Explore phase (flood-fill) followed by sprint phase (BFS shortest path).
 *   Never returns under normal operation.
 */
void maze_run(void);

/**
 * maze_trigger_sprint()
 *   Called from the blue-button ISR.
 *   If the goal has been reached, immediately begins the sprint run.
 */
void maze_trigger_sprint(void);

#ifdef __cplusplus
}
#endif

#endif /* MAZE_H */
