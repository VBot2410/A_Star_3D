set(GTEST_SHUFFLE 1)

add_executable(
    A_Star-test
    main.cpp
    A_Star_Test.cpp
    ../app/Build_Map.cpp
    ../app/Planner.cpp
)

target_include_directories(A_Star-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(A_Star-test PUBLIC gtest)
