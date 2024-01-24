include(CTest)
find_package(GTest CONFIG REQUIRED)

set(test_list
    service_manager
    activity_metric
)

foreach(file IN LISTS test_list)
    add_executable(${file}_test ${SOURCES} tests/${file}.cpp src/${file}.cpp)
    target_compile_options(${file}_test PRIVATE -Wall -Wshadow -Wextra -fno-access-control)
    target_link_libraries(${file}_test GTest::gtest)
    add_test(NAME ${file}_test COMMAND ${file}_test)
endforeach()
