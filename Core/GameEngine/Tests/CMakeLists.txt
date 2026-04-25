enable_testing()

add_executable(GameEnginePathfindTests
    Pathfind/PathfindTestMain.cpp
    Pathfind/PathfindHeaderCompatibilityTest.cpp
)

target_include_directories(GameEnginePathfindTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../Include
)

target_link_libraries(GameEnginePathfindTests PRIVATE
    corei_always
    corei_gameengine_include
)

add_test(
    NAME GameEnginePathfindTests
    COMMAND GameEnginePathfindTests
)
