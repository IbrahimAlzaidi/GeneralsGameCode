#include <cstdio>

#include "PathfindTestAssertions.h"

int RunHeaderCompatibilityTests();

int main() {
    std::printf("GameEngine Pathfinder Tests\n");
    std::printf("============================\n\n");

    RunHeaderCompatibilityTests();

    return PathfindTest::PrintSummary();
}
