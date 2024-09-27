BUILD = DEBUG
#BUILD = RELEASE
CXX = mpic++
N = 2
nstrings = 100

CXXFLAGS = -O3 -DNDEBUG
ifeq ($(BUILD), DEBUG)
    CXXFLAGS = -g -O0
endif

all: solver_mpi libtimer.so

solver_mpi: A.o libtimer.so
	$(CXX) -o solver_mpi A.o -L. -ltimer '-Wl,-rpath,$$ORIGIN'
#solver_mpi: A.o timer.o
#	$(CXX) -o solver_mpi A.o timer.o
libtimer.so: timer.o
	$(CXX) -o libtimer.so timer.o -shared
A.o: Application6.cpp timer.hpp
	$(CXX) -o A.o $(CXXFLAGS) -c Application6.cpp
timer.o: timer.cpp timer.hpp
	$(CXX) -o timer.o $(CXXFLAGS) -fPIC -c timer.cpp
clean:
	rm -f solver_mpi A.o timer.o libtimer.so
run: solver_mpi
	mpirun -np $(N) ./solver_mpi $(nstrings)
