cmake_minimum_required(VERSION 3.1)

project(polymorphic-list)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(external/GSL/include include)

option(POLYMORPHIC_LIST_BUILD_TESTS "Build tests for polymorphic-list." ON)
if(POLYMORPHIC_LIST_BUILD_TESTS)
	include_directories(external/Catch2/single_include)

	add_executable(test test/main.spec.cpp test/plist.spec.cpp)
endif()

option(POLYMORPHIC_LIST_BUILD_BENCHMARKS
	   "Build benchmarks for polymorphic-list." OFF)
if(POLYMORPHIC_LIST_BUILD_BENCHMARKS)
	find_package(Threads REQUIRED)
	find_package(benchmark REQUIRED)

	include_directories(${benchmark_INCLUDE_DIRS})

	add_executable(bench bench/main.cpp bench/plist.cpp)
	target_link_libraries(bench benchmark benchmark_main Threads::Threads)
endif()
