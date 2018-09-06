COMPILER = g++
CCFLAGS = -std=c++11 -lkrpc -lprotobuf -lpthread

launcher:
	${COMPILER} launcher.cpp -o launch.out ${CCFLAGS}

