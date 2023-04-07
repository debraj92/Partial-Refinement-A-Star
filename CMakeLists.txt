cmake_minimum_required(VERSION 3.19)
project(658Project)

set(CMAKE_CXX_STANDARD 20)
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_executable(658Project main.cpp RealWorld.cpp RealWorld.h AbstractNode.h AbstractGraph.cpp AbstractGraph.h AStarOpenList.h
        AstarOpenList.cpp AStarSearch.cpp AStarSearch.h DataGeneratorForExperiments.cpp DataGeneratorForExperiments.h DataPoint.h AbstractGraph_2.cpp AbstractGraph_2.h AbstractGraph_3.cpp AbstractGraph_3.h Abstraction.h AbstractGraph_4.cpp AbstractGraph_4.h AbstractGraph_5.cpp AbstractGraph_5.h)
