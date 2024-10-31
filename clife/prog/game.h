
#ifndef GAME_H
#define GAME_H

#include "board.h"
#include <stdbool.h>
#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

    typedef struct Engine Engine;
    typedef struct Viewer Viewer;
    typedef struct Driver Driver;

    typedef struct
    {
        void (*step)(const Engine* engine, const Board* board, Board* new_board);
    } EngineInterface;

    typedef struct
    {
        void (*display)(const Viewer* viewer, const Board* board);
        void (*game_over)(const Viewer* viewer);
    } ViewerInterface;

    struct Engine
    {
        EngineInterface* interface;
    };

    struct Viewer
    {
        ViewerInterface* interface;
        int _ncols;
        int _nrows;
    };

    struct Driver
    {
        char* arg;
        Board* board;
        Engine* engine;
        Viewer* viewer;
    };

    void initialize_game(Driver* driver);
    void cleanup_game(Driver* driver);
    bool is_over(Driver* driver, Board* new_board);

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // GAME_H