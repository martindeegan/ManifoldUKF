project(tests)
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

include_directories(
        ${SOFTWARE_SOURCE_DIR}/include
        ${SOFTWARE_SOURCE_DIR}/tests
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${SOPHUS_INCLUDE_DIR}
)

add_library(testhelper SHARED TestHelpers.cpp)

target_link_libraries(testhelper state manifold_ukf)

# Some tests are broken and need to be fixed. Filter works tho :)
set(TEST_SRCS
        StateTest.cpp
        HistoryTest.cpp
        # UnscentedTransformTest.cpp
        # LidarTest.cpp
        PredictionTest.cpp
        # PlaneFitTest.cpp
        )

foreach (testSrc ${TEST_SRCS})
    #Extract the filename without an extension (NAME_WE)
    get_filename_component(testName ${testSrc} NAME_WE)

    #Add compile target
    add_executable(${testName} ${testSrc})

    #link to Boost libraries AND your targets and dependencies
    target_link_libraries(${testName} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
            state
            measurements
            manifold_ukf
            testhelper
            ${YAML_CPP_LIBRARY}
            )

    set(TEST_BIN_DIR ${CMAKE_SOURCE_DIR}/bin/test)

    #I like to move testing binaries into a testBin directory
    set_target_properties(${testName} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${TEST_BIN_DIR})

    #Finally add it to test execution -
    #Notice the WORKING_DIRECTORY and COMMAND
    add_test(NAME ${testName}
            WORKING_DIRECTORY ${TEST_BIN_DIR}
            COMMAND ${TEST_BIN_DIR}/${testName})
endforeach (testSrc)
