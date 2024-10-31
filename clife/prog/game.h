
#ifndef GAME_H
#define GAME_H

#include "arguments.hpp"
#include "board.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
        EngineInterface* inter;
    };

    struct Viewer
    {
        ViewerInterface* inter;
        int _ncols;
        int _nrows;
    };

    struct Driver
    {
        int delay;
        Board* board;
        Engine* engine;
        Viewer* viewer;
    };

    void initialize_game(Driver* driver, Arguments& arg);
    void run_game(Driver driver);

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // GAME_H