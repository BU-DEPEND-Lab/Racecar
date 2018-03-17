g++ -std=c++0x -ggdb `pkg-config --cflags opencv` -o `basename AHS.cpp .cpp` AHS.cpp `pkg-config --libs opencv`
