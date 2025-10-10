#ifndef MAZEGRAPH_H
#define MAZEGRAPH_H

#include <string>
#include <vector>

#include "MazeNode.h"

class MazeGraph
{
  public:
    MazeGraph(int mazeNumRows, int mazeNumCols);
    ~MazeGraph();

    MazeNode*              getNode(int nodePosX, int nodePosY);
    std::vector<MazeNode*> getNodeNeighbors(MazeNode* node, bool includeDiagNeighbors = false);

    int  getMazeWidth();
    int  getMazeHeight();
    void printASCIIMaze();

  private:
    void createCellsInMaze();
    void linkNeighboringCellsInMaze();

    std::string stringifyTopMazeEdge();
    std::string stringifyMazeRow(int rowIndex);

    std::vector<std::vector<MazeNode*>> cellMaze;
};

#endif