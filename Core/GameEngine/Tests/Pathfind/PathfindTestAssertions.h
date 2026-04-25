#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace PathfindTest {

inline int g_testsRun = 0;
inline int g_testsPassed = 0;
inline int g_testsFailed = 0;

#define PATHFIND_TEST_ASSERT(condition, message)                         \
    do {                                                                  \
        if (!(condition)) {                                               \
            std::printf("  FAIL: %s (at %s:%d)\n", message, __FILE__, __LINE__); \
            return 1;                                                     \
        }                                                                 \
    } while (0)

#define PATHFIND_TEST_ASSERT_EQ(actual, expected, message)               \
    do {                                                                  \
        if ((actual) != (expected)) {                                     \
            std::printf("  FAIL: %s (expected %d, got %d, at %s:%d)\n",  \
                message, static_cast<int>(expected), static_cast<int>(actual), \
                __FILE__, __LINE__);                                       \
            return 1;                                                     \
        }                                                                 \
    } while (0)

#define PATHFIND_TEST_ASSERT_TRUE(condition, message)                    \
    PATHFIND_TEST_ASSERT(condition, message)

#define PATHFIND_TEST_ASSERT_FALSE(condition, message)                   \
    PATHFIND_TEST_ASSERT(!(condition), message)

inline void RecordTestResult(const char *name, int result) {
    g_testsRun++;
    if (result == 0) {
        g_testsPassed++;
        std::printf("  PASS: %s\n", name);
    } else {
        g_testsFailed++;
    }
}

inline int PrintSummary() {
    std::printf("\nResults: %d/%d passed", g_testsPassed, g_testsRun);
    if (g_testsFailed > 0) {
        std::printf(", %d FAILED", g_testsFailed);
    }
    std::printf("\n");
    return g_testsFailed > 0 ? 1 : 0;
}

}
