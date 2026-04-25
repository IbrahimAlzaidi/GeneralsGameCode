#include "GameLogic/AIPathfind.h"

#include "PathfindTestAssertions.h"

static int TestPublicTypesExist() {
    PathNode *pathNode = nullptr;
    (void)pathNode;

    Path *path = nullptr;
    (void)path;

    PathfindCell *cell = nullptr;
    (void)cell;

    PathfindServicesInterface *psi = nullptr;
    (void)psi;

    return 0;
}

static int TestPublicConstantsExist() {
    volatile auto closeEnough = PATHFIND_CLOSE_ENOUGH;
    (void)closeEnough;

    volatile auto maxPriority = PATH_MAX_PRIORITY;
    (void)maxPriority;

    volatile auto cellSize = PATHFIND_CELL_SIZE;
    (void)cellSize;

    return 0;
}

static int TestPublicEnumsExist() {
    PathfindCell::CellType clearType = PathfindCell::CELL_CLEAR;
    PathfindCell::CellType waterType = PathfindCell::CELL_WATER;
    PathfindCell::CellType cliffType = PathfindCell::CELL_CLIFF;
    PathfindCell::CellType obstacleType = PathfindCell::CELL_OBSTACLE;
    PathfindCell::CellType impassableType = PathfindCell::CELL_IMPASSABLE;
    (void)clearType; (void)waterType; (void)cliffType;
    (void)obstacleType; (void)impassableType;

    PathfindCell::CellFlags noUnits = PathfindCell::NO_UNITS;
    PathfindCell::CellFlags unitGoal = PathfindCell::UNIT_GOAL;
    (void)noUnits; (void)unitGoal;

    return 0;
}

static int TestPublicLayerConstants() {
    volatile auto layerGround = LAYER_GROUND;
    volatile auto layerInvalid = LAYER_INVALID;
    volatile auto layerTop = LAYER_TOP;
    (void)layerGround; (void)layerInvalid; (void)layerTop;

    return 0;
}

int RunHeaderCompatibilityTests() {
    int result = 0;

    std::printf("Header compatibility tests:\n");

    result = TestPublicTypesExist();
    PathfindTest::RecordTestResult("Public types exist", result);

    result = TestPublicConstantsExist();
    PathfindTest::RecordTestResult("Public constants exist", result);

    result = TestPublicEnumsExist();
    PathfindTest::RecordTestResult("Public enums exist", result);

    result = TestPublicLayerConstants();
    PathfindTest::RecordTestResult("Public layer constants exist", result);

    return 0;
}
