/* ============================================================
 * maze.c  —  Flood-Fill Maze Solver
 * STM32F446RE · BTS7960 · MPU6050 · 3× HC-SR04
 *
 * Algorithm
 * ─────────
 *  Phase 1 – EXPLORE
 *    Modified flood-fill: always move to the unvisited neighbour
 *    with the lowest flood distance.  Walls are sensed at each cell
 *    and updated bidirectionally.  Flood is re-run after every new
 *    wall discovery.  Ends when every reachable cell is visited.
 *
 *  Phase 2 – SPRINT
 *    Classic BFS from start to goal on the fully-known map.
 *    Robot follows the BFS path at higher speed.
 *
 * Coordinate convention
 * ─────────────────────
 *    (col=0, row=0) = start corner (bottom-left)
 *    col increases East, row increases North
 *    Robot heading: N=0, E=1, S=2, W=3  (matches Direction enum)
 * ============================================================ */

#include "maze.h"
#include "main.h"       /* pin defines, HAL handles             */
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* ============================================================
   EXTERNAL FUNCTIONS  (defined in main.c)
   ============================================================ */
extern void  move_distance(float distance_mm, float speed_mms);
extern void  rotate(float angle_deg, float speed_mms);
extern void  stop_motors(void);
extern void  debug_print(const char* msg);
extern float get_front_ultrasound(void);
extern float get_left_ultrasound(void);
extern float get_right_ultrasound(void);

/* ============================================================
   TUNING PARAMETERS
   ============================================================ */
#define EXPLORE_SPEED_MMS   200.0f   /* mm/s while mapping      */
#define SPRINT_SPEED_MMS    350.0f   /* mm/s on known path      */
#define TURN_SPEED_MMS      180.0f   /* mm/s during rotations   */
#define WALL_THRESHOLD_MM   180.0f   /* sensor < this = wall    */
#define FRONT_THRESHOLD_MM  160.0f   /* slightly tighter front  */

#define INF_DIST    255             /* unreachable sentinel     */

/* ============================================================
   MAZE DATA
   ============================================================ */
static uint8_t  walls[MAZE_H][MAZE_W];   /* wall bitmask per cell  */
static uint8_t  dist [MAZE_H][MAZE_W];   /* flood distances        */

/* ============================================================
   ROBOT STATE
   ============================================================ */
static int8_t   robot_col;
static int8_t   robot_row;
static Direction robot_dir;

/* sprint trigger flag  (set by ISR) */
static volatile uint8_t sprint_requested = 0;
/* set once the goal has been found */
static uint8_t goal_found = 0;

/* ============================================================
   DIRECTION HELPERS
   ============================================================ */

/* delta-col / delta-row for each cardinal direction */
static const int8_t DC[4] = {  0, +1,  0, -1 }; /* N E S W */
static const int8_t DR[4] = { +1,  0, -1,  0 };

/* wall bit associated with each cardinal direction */
static const uint8_t DIR_WALL[4] = { WALL_N, WALL_E, WALL_S, WALL_W };

/* opposite direction */
static inline Direction opposite(Direction d) { return (Direction)((d + 2) & 3); }

/* CW rotation amount in degrees to turn from `from` to `to` (signed) */
static int16_t turn_deg(Direction from, Direction to)
{
    int16_t diff = (int16_t)to - (int16_t)from;   /* -3 … +3 */
    if (diff ==  2) return  180;
    if (diff == -2) return -180;
    if (diff ==  3) diff = -1;
    if (diff == -3) diff =  3;
    return (int16_t)(diff * 90);
}

/* Is cell (c,r) inside the grid? */
static inline uint8_t in_grid(int8_t c, int8_t r)
{
    return (c >= 0 && c < MAZE_W && r >= 0 && r < MAZE_H);
}

/* Is cell (c,r) the goal? */
static inline uint8_t is_goal(int8_t c, int8_t r)
{
    return (c == GOAL_C0 && r == GOAL_R0);
}

/* ============================================================
   FLOOD FILL  (BFS from goal outward)
   Sets dist[r][c] = BFS distance to goal through open passages.
   Cells separated by walls are not connected.
   ============================================================ */
static void flood_fill(void)
{
    /* Init all to INF */
    memset(dist, INF_DIST, sizeof(dist));

    /* Simple BFS queue — 7×7 = 49 cells max */
    uint8_t qc[MAZE_H * MAZE_W];
    uint8_t qr[MAZE_H * MAZE_W];
    uint8_t head = 0, tail = 0;

    /* Seed: goal cell = 0 */
    dist[GOAL_R0][GOAL_C0] = 0;
    qc[tail] = GOAL_C0;
    qr[tail] = GOAL_R0;
    tail++;

    while (head != tail)
    {
        uint8_t cc = qc[head];
        uint8_t cr = qr[head];
        head++;

        uint8_t next_d = dist[cr][cc] + 1;

        for (Direction d = DIR_N; d <= DIR_W; d++)
        {
            /* Skip if wall exists in this direction */
            if (walls[cr][cc] & DIR_WALL[d]) continue;

            int8_t nc = (int8_t)cc + DC[d];
            int8_t nr = (int8_t)cr + DR[d];

            if (!in_grid(nc, nr)) continue;
            if (dist[nr][nc] != INF_DIST) continue;

            dist[nr][nc] = next_d;
            qc[tail] = (uint8_t)nc;
            qr[tail] = (uint8_t)nr;
            tail++;
        }
    }
}

/* ============================================================
   WALL INITIALISATION
   Border walls + interior cleared.
   ============================================================ */
static void init_walls(void)
{
    memset(walls, 0, sizeof(walls));

    for (int8_t r = 0; r < MAZE_H; r++)
    {
        for (int8_t c = 0; c < MAZE_W; c++)
        {
            if (r == MAZE_H - 1) walls[r][c] |= WALL_N;
            if (c == MAZE_W - 1) walls[r][c] |= WALL_E;
            if (r == 0)          walls[r][c] |= WALL_S;
            if (c == 0)          walls[r][c] |= WALL_W;
        }
    }
}

/* ============================================================
   RECORD WALL  — bidirectional update
   ============================================================ */
static void record_wall(int8_t c, int8_t r, Direction d)
{
    if (!in_grid(c, r)) return;
    walls[r][c] |= DIR_WALL[d];

    /* mirror on neighbour */
    int8_t nc = c + DC[d];
    int8_t nr = r + DR[d];
    if (in_grid(nc, nr))
        walls[nr][nc] |= DIR_WALL[opposite(d)];
}

/* ============================================================
   SENSE WALLS from current position
   Reads all three ultrasonic sensors and updates the map.
   ============================================================ */
static void sense_walls(void)
{
    float f = get_front_ultrasound();
    float l = get_left_ultrasound();
    float rr = get_right_ultrasound();

    /* Front sensor → wall ahead in robot_dir */
    if (f > 0.0f && f < FRONT_THRESHOLD_MM)
        record_wall(robot_col, robot_row, robot_dir);

    /* Left sensor → wall to the left of robot_dir */
    Direction left_dir = (Direction)((robot_dir + 3) & 3);
    if (l > 0.0f && l < WALL_THRESHOLD_MM)
        record_wall(robot_col, robot_row, left_dir);

    /* Right sensor → wall to the right of robot_dir */
    Direction right_dir = (Direction)((robot_dir + 1) & 3);
    if (rr > 0.0f && rr < WALL_THRESHOLD_MM)
        record_wall(robot_col, robot_row, right_dir);

    /* Mark back wall as open (we came from there) */
    /* (don't force it closed — leave it as sensed) */
}

/* ============================================================
   CHOOSE NEXT DIRECTION (explore)
   Returns the direction with the lowest flood distance
   among open, in-grid neighbours.  Prefers unvisited cells.
   ============================================================ */
static Direction choose_next_dir(void)
{
    uint8_t  best_d   = INF_DIST;
    uint8_t  best_d_v = INF_DIST;   /* best among visited   */
    Direction best    = DIR_N;       /* default (overwritten) */
    Direction best_v  = DIR_N;
    uint8_t  found    = 0;
    uint8_t  found_v  = 0;

    for (Direction d = DIR_N; d <= DIR_W; d++)
    {
        if (walls[robot_row][robot_col] & DIR_WALL[d]) continue;

        int8_t nc = robot_col + DC[d];
        int8_t nr = robot_row + DR[d];
        if (!in_grid(nc, nr)) continue;

        uint8_t nd = dist[nr][nc];

        /* Prefer unvisited */
        if (!(walls[nr][nc] & WALL_VISITED))
        {
            if (nd < best_d) { best_d = nd; best = d; found = 1; }
        }
        else
        {
            if (nd < best_d_v) { best_d_v = nd; best_v = d; found_v = 1; }
        }
    }

    if (found)   return best;
    if (found_v) return best_v;
    return robot_dir; /* no reachable neighbour — stuck */
}

/* ============================================================
   PHYSICAL MOVEMENT HELPERS
   ============================================================ */

static void face_direction(Direction target, float spd)
{
    int16_t deg = turn_deg(robot_dir, target);
    if (deg != 0)
        rotate((float)deg, spd);
    robot_dir = target;
}

static void advance_one_cell(float spd)
{
    move_distance(CELL_MM, spd);
    robot_col += DC[robot_dir];
    robot_row += DR[robot_dir];
}

/* Debug helper: print current position */
static void print_pos(void)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "POS(%d,%d) DIR=%d DIST=%d\r\n",
             robot_col, robot_row, (int)robot_dir,
             dist[robot_row][robot_col]);
    debug_print(buf);
}

/* ============================================================
   COUNT UNVISITED REACHABLE CELLS
   ============================================================ */
static uint8_t unvisited_reachable(void)
{
    uint8_t count = 0;
    for (int8_t r = 0; r < MAZE_H; r++)
        for (int8_t c = 0; c < MAZE_W; c++)
            if (!(walls[r][c] & WALL_VISITED) && dist[r][c] != INF_DIST)
                count++;
    return count;
}

/* ============================================================
   EXPLORE PHASE
   ============================================================ */
static void explore(void)
{
    debug_print("EXPLORE start\r\n");

    while (1)
    {
        /* 1. Mark current cell visited */
        walls[robot_row][robot_col] |= WALL_VISITED;

        /* 2. Detect goal */
        if (is_goal(robot_col, robot_row))
        {
            goal_found = 1;
            debug_print("GOAL reached!\r\n");
        }

        /* 3. Sense walls around current cell */
        sense_walls();

        /* 4. Re-flood after potential new walls */
        flood_fill();

        /* 5. Check sprint request or full coverage */
        if (sprint_requested && goal_found) break;
        if (unvisited_reachable() == 0)     break;

        /* 6. Choose best neighbour */
        Direction next = choose_next_dir();

        /* Safety: if flood says current is INF and no better option */
        if (dist[robot_row][robot_col] == INF_DIST)
        {
            debug_print("STUCK — flood blocked\r\n");
            break;
        }

        /* 7. Turn + move */
        face_direction(next, TURN_SPEED_MMS);
        advance_one_cell(EXPLORE_SPEED_MMS);
        print_pos();
    }

    debug_print("EXPLORE done\r\n");
}

/* ============================================================
   BFS PATH  (sprint)
   Returns path length, path stored in path_dirs[].
   ============================================================ */
#define MAX_PATH (MAZE_W * MAZE_H)
static Direction path_dirs[MAX_PATH];
static uint8_t   path_len = 0;

static uint8_t build_sprint_path(void)
{
    /* Re-run flood from goal to ensure distances are current */
    flood_fill();

    path_len = 0;

    int8_t cc = robot_col;
    int8_t cr = robot_row;

    if (dist[cr][cc] == INF_DIST)
    {
        debug_print("SPRINT: goal unreachable!\r\n");
        return 0;
    }

    while (!is_goal(cc, cr))
    {
        uint8_t   best_d = INF_DIST;
        Direction best   = DIR_N;
        uint8_t   found  = 0;

        for (Direction d = DIR_N; d <= DIR_W; d++)
        {
            if (walls[cr][cc] & DIR_WALL[d]) continue;
            int8_t nc = cc + DC[d];
            int8_t nr = cr + DR[d];
            if (!in_grid(nc, nr)) continue;
            if (dist[nr][nc] < best_d)
            {
                best_d = dist[nr][nc];
                best   = d;
                found  = 1;
            }
        }

        if (!found || path_len >= MAX_PATH - 1)
        {
            debug_print("SPRINT: path broken!\r\n");
            return 0;
        }

        path_dirs[path_len++] = best;
        cc += DC[best];
        cr += DR[best];
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "SPRINT path len=%d\r\n", path_len);
    debug_print(buf);
    return 1;
}

/* ============================================================
   SPRINT PHASE
   ============================================================ */
static void sprint(void)
{
    debug_print("SPRINT start\r\n");

    if (!build_sprint_path()) return;

    for (uint8_t i = 0; i < path_len; i++)
    {
        face_direction(path_dirs[i], TURN_SPEED_MMS);
        advance_one_cell(SPRINT_SPEED_MMS);
        print_pos();
    }

    stop_motors();
    debug_print("SPRINT done — GOAL!\r\n");
}

/* ============================================================
   PUBLIC API
   ============================================================ */

void maze_init(void)
{
    init_walls();
    flood_fill();

    robot_col = 0;
    robot_row = 0;
    robot_dir = DIR_N;

    sprint_requested = 0;
    goal_found       = 0;

    debug_print("MAZE init OK\r\n");
}

void maze_run(void)
{
    explore();

    /* Return to start before sprinting */
    debug_print("Returning to start...\r\n");

    /* Re-seed flood from START to navigate back */
    /* We temporarily redirect flood to start    */
    /* Hack: swap goal to (0,0), flood, navigate, restore */
    /* Simpler: just reverse the sprint path back         */

    /* --- navigate back to (0,0) using flood toward start --- */
    /* Re-flood from (0,0) */
    memset(dist, INF_DIST, sizeof(dist));
    {
        uint8_t qc[MAZE_H * MAZE_W];
        uint8_t qr[MAZE_H * MAZE_W];
        uint8_t head = 0, tail = 0;
        dist[0][0] = 0;
        qc[tail] = 0; qr[tail] = 0; tail++;
        while (head != tail)
        {
            uint8_t cc = qc[head], cr = qr[head]; head++;
            uint8_t nd = dist[cr][cc] + 1;
            for (Direction d = DIR_N; d <= DIR_W; d++)
            {
                if (walls[cr][cc] & DIR_WALL[d]) continue;
                int8_t nc = (int8_t)cc + DC[d];
                int8_t nr = (int8_t)cr + DR[d];
                if (!in_grid(nc, nr) || dist[nr][nc] != INF_DIST) continue;
                dist[nr][nc] = nd;
                qc[tail] = (uint8_t)nc; qr[tail] = (uint8_t)nr; tail++;
            }
        }
    }

    /* Follow gradient back to start */
    while (robot_col != 0 || robot_row != 0)
    {
        uint8_t   best_d = INF_DIST;
        Direction best   = DIR_N;
        uint8_t   found  = 0;
        for (Direction d = DIR_N; d <= DIR_W; d++)
        {
            if (walls[robot_row][robot_col] & DIR_WALL[d]) continue;
            int8_t nc = robot_col + DC[d];
            int8_t nr = robot_row + DR[d];
            if (!in_grid(nc, nr)) continue;
            if (dist[nr][nc] < best_d) { best_d = dist[nr][nc]; best = d; found = 1; }
        }
        if (!found) break;
        face_direction(best, TURN_SPEED_MMS);
        advance_one_cell(EXPLORE_SPEED_MMS);
    }
    debug_print("Back at start\r\n");

    /* Wait for button or proceed automatically */
    if (!sprint_requested)
    {
        debug_print("Waiting for button...\r\n");
        while (!sprint_requested) { HAL_Delay(50); }
    }

    /* Restore flood from goal for sprint */
    flood_fill();

    sprint();

    /* Infinite blink to signal completion */
    while (1)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(200);
    }
}

void maze_trigger_sprint(void)
{
    sprint_requested = 1;
}
