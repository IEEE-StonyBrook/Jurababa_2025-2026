CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++11 -I./Robot -I./Solver -I./Virtual -I./Utils -g -O0
CXXFLAGS = -Wall -Wextra -std=c++11 -I./Robot -I./Solver -I./Virtual -I./Utils -g -O0

OUTDIR = bin
OUT = $(OUTDIR)/Micromouse.out

# Make sure output directory exists
$(shell mkdir -p $(OUTDIR))

# Source file groups
VIRTUAL_SRCS = Src/Maze/InternalMouse.cpp Src/Maze/MazeGraph.cpp Src/Maze/MazeNode.cpp
UTILS_SRC = Src/Common/LogSystem.cpp
SOLVER_SRCS = Src/Navigation/AStarSolver.cpp Src/Navigation/PathConverter.cpp
MAIN_SRCS = Src/Platform/Simulator/API.cpp Src/Main.cpp

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