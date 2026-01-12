# Micromouse Simulator Makefile
# Builds mms-compatible simulator binary using firmware algorithm code

CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -DSIMULATOR_BUILD -I../firmware -g -O0

OUTDIR = bin
OUT = $(OUTDIR)/Micromouse.out

# Simulator main
SIM_SRCS = main.cpp

# Firmware sources (algorithm code only, no hardware)
MAZE_SRCS = ../firmware/maze/maze.cpp ../firmware/maze/mouse.cpp
NAV_SRCS = ../firmware/navigation/flood_fill.cpp \
           ../firmware/navigation/a_star.cpp \
           ../firmware/navigation/path_utils.cpp \
           ../firmware/navigation/path_converter.cpp \
           ../firmware/navigation/diagonalizer.cpp
APP_SRCS = ../firmware/app/api.cpp
COMMON_SRCS = ../firmware/common/log.cpp

# All sources
SRCS = $(SIM_SRCS) $(MAZE_SRCS) $(NAV_SRCS) $(APP_SRCS) $(COMMON_SRCS)

# Object files
OBJS = $(patsubst %.cpp,$(OUTDIR)/%.o,$(notdir $(SRCS)))

# VPATH for finding source files
VPATH = .:../firmware/maze:../firmware/navigation:../firmware/app:../firmware/common

# Default target
all: $(OUT)

# Create output directory
$(OUTDIR):
	mkdir -p $(OUTDIR)

# Linking
$(OUT): $(OBJS) | $(OUTDIR)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Compilation
$(OUTDIR)/%.o: %.cpp | $(OUTDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean
clean:
	rm -rf $(OUTDIR)

# Run (for local testing, not mms)
run: all
	./$(OUT)

# Generate compile_commands.json for IDE support (requires bear: brew install bear)
compile_commands.json: clean
	bear -- $(MAKE) all

.PHONY: all clean run
