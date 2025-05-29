#include "MazeGraph.h"

#include <sstream>

MazeGraph::MazeGraph(int mazeNumRows, int mazeNumCols) {
  // Array of (# of rows) copies of (array of (# of cols) maze cells).
  cellMaze = std::vector<std::vector<MazeNode*>>(
      mazeNumCols, std::vector<MazeNode*>(mazeNumRows));

  createCellsInMaze();
  linkNeighboringCellsInMaze();
}

MazeGraph::~MazeGraph() {
  // Deletes all cell objects in the maze.
  for (int col = 0; col < cellMaze.size(); col++) {
    for (int row = 0; row < cellMaze[0].size(); row++) {
      delete cellMaze[col][row];
    }
  }
}

void MazeGraph::createCellsInMaze() {
  // Formatted as (x, y); x = column, y = row number.
  for (int col = 0; col < cellMaze.size(); col++) {
    for (int row = 0; row < cellMaze[0].size(); row++) {
      cellMaze[col][row] = new MazeNode(col, row);
    }
  }
}

void MazeGraph::linkNeighboringCellsInMaze() {
  // Adds all the neighbors to each cell.
  int colSize = cellMaze.size();
  int rowSize = cellMaze[0].size();

  for (int col = 0; col < colSize; col++) {
    for (int row = 0; row < rowSize; row++) {
      MazeNode* currentCell = cellMaze[col][row];

      if (row < rowSize - 1)
        currentCell->setCellInDirection(cellMaze[col][row + 1], 'N');
      if (col < colSize - 1)
        currentCell->setCellInDirection(cellMaze[col + 1][row], 'E');
      if (row > 0) currentCell->setCellInDirection(cellMaze[col][row - 1], 'S');
      if (col > 0) currentCell->setCellInDirection(cellMaze[col - 1][row], 'W');
    }
  }
}

MazeNode* MazeGraph::getNode(int nodePosX, int nodePosY) {
  return cellMaze[nodePosX][nodePosY];
}

void MazeGraph::printASCIIMaze() {
  std::stringstream asciiMaze;
  asciiMaze << "ASCII Maze:\n";

  // Adds top edge of maze.
  asciiMaze << stringifyTopMazeEdge();

  // Adds all rows to the string top --> bottom.
  for (int row = cellMaze[0].size() - 1; row >= 0; row--) {
    asciiMaze << stringifyMazeRow(row);
  }
}

std::string MazeGraph::stringifyTopMazeEdge() {
  std::stringstream topEdgeString;

  for (int col = 0; col < cellMaze.size(); col++) {
    topEdgeString << "+---";
  }
  topEdgeString << "+\n";
  return topEdgeString.str();
}

std::string MazeGraph::stringifyMazeRow(int rowIndex) {
  std::stringstream rowString, verticalWallString, horizontalWallString;

  verticalWallString << "|";
  for (int col = 0; col < cellMaze.size(); col++) {
    MazeNode* currentCell = cellMaze[col][rowIndex];
    // Adds vertical walls if east wall exists for each cell in row.
    if (currentCell->getIsWall('E'))
      verticalWallString << "   |";
    else
      verticalWallString << "    ";

    // Adds horizontal walls if south wall exists for each cell in row.
    if (currentCell->getIsWall('S'))
      horizontalWallString << "+---";
    else
      horizontalWallString << "+   ";
  }

  rowString << verticalWallString.str() << "\n" << horizontalWallString.str();
  return rowString.str();
}

std::vector<MazeNode*> MazeGraph::getNodeNeighbors(MazeNode* node,
                                                   bool includeDiagNeighbors) {
  std::vector<MazeNode*> neighbors;
  int x = node->getCellXPos();
  int y = node->getCellYPos();
  int w = getMazeWidth();
  int h = getMazeHeight();

  // Cardinal directions
  if (x > 0) neighbors.push_back(cellMaze[x - 1][y]);      // West
  if (x < w - 1) neighbors.push_back(cellMaze[x + 1][y]);  // East
  if (y > 0) neighbors.push_back(cellMaze[x][y - 1]);      // South
  if (y < h - 1) neighbors.push_back(cellMaze[x][y + 1]);  // North

  // Diagonal directions
  if (includeDiagNeighbors) {
    if (x > 0 && y > 0) neighbors.push_back(cellMaze[x - 1][y - 1]);      // SW
    if (x > 0 && y < h - 1) neighbors.push_back(cellMaze[x - 1][y + 1]);  // NW
    if (x < w - 1 && y > 0) neighbors.push_back(cellMaze[x + 1][y - 1]);  // SE
    if (x < w - 1 && y < h - 1)
      neighbors.push_back(cellMaze[x + 1][y + 1]);  // NE
  }

  return neighbors;
}

int MazeGraph::getMazeWidth() { return cellMaze.size(); }

int MazeGraph::getMazeHeight() { return cellMaze[0].size(); }