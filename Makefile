CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++11 -I./Robot -I./Solver -I./Virtual -I./Utils -g -O0

OUTDIR = bin
OUT = $(OUTDIR)/Micromouse.out

# Make sure output directory exists
$(shell mkdir -p $(OUTDIR))

# Source file groups
VIRTUAL_SRCS = src/Maze/InternalMouse.cpp src/Maze/MazeGraph.cpp src/Maze/MazeNode.cpp
UTILS_SRC = src/Common/LogSystem.cpp
SOLVER_SRCS = src/Navigation/AStarSolver.cpp src/Navigation/PathConverter.cpp src/Navigation/Diagonalizer.cpp src/Navigation/FrontierBasedSearchSolver.cpp
MAIN_SRCS = src/Platform/Simulator/API.cpp src/Main_Simulator.cpp src/PathUtils.cpp

# Combine all sources
SRCS = $(VIRTUAL_SRCS) $(UTILS_SRC) $(SOLVER_SRCS) $(MAIN_SRCS)

# Object files go in SimOutput/
OBJS = $(patsubst %.cpp,$(OUTDIR)/%.o,$(SRCS))

# Default target
all: $(OUT)

# Linking
$(OUT): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Compilation rule
$(OUTDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Cleanup
clean:
	rm -rf $(OUTDIR)

# Run
run: all
	./$(OUT)