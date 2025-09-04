CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++11 -I./Robot -I./Solver -I./Virtual -I./Utils

OUTDIR = out
OUT = $(OUTDIR)/Micromouse.out

# Make sure output directory exists
$(shell mkdir -p $(OUTDIR))

# Source file groups
VIRTUAL_SRCS = src/Virtual/Mouse/InternalMouse.cpp src/Virtual/Mouse/MazeGraph.cpp src/Virtual/Mouse/MazeNode.cpp
UTILS_SRC = src/Utils/LogSystem.cpp
SOLVER_SRCS = src/Virtual/Solver/AStarSolver.cpp
MAIN_SRCS = src/API.cpp src/Main.cpp

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