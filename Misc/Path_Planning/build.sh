g++ -std=c++0x -ggdb `pkg-config --cflags opencv` -o `basename Astar.cpp .cpp` Astar.cpp `pkg-config --libs opencv`
g++ -std=c++0x -ggdb `pkg-config --cflags opencv` -o `basename AHS.cpp .cpp` AHS.cpp `pkg-config --libs opencv`
g++ -std=c++0x -ggdb `pkg-config --cflags opencv` -o `basename RRT.cpp .cpp` RRT.cpp `pkg-config --libs opencv`
g++ -std=c++0x -ggdb `pkg-config --cflags opencv` -o `basename PRMDijkstra.cpp .cpp` PRMDijkstra.cpp `pkg-config --libs opencv`
