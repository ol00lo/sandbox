#ifndef GAME_H
#define GAME_H

#include "arguments.hpp"
#include "board.hpp"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct Engine Engine;
    typedef struct Viewer Viewer;
    typedef struct Driver Driver;

    Engine* engine_create();
    void engine_destroy(Engine* engine);
    Board engine_step(Engine* engine, const Board* board);

    Viewer* viewer_create();
    void viewer_destroy(Viewer* viewer);
    void viewer_display(Viewer* viewer, const Board* board);
    void viewer_game_over(Viewer* viewer);

    Driver* driver_create(const Arguments* arg);
    void driver_destroy(Driver* driver);
    void driver_start(Driver* driver);
    int driver_is_over(Driver* driver, const Board* new_board);
    int driver_next(Driver* driver);

#ifdef __cplusplus
}
#endif

#endif // GAME_H