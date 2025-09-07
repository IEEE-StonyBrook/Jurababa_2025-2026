#ifndef PATHCONVERTER_H
#define PATHCONVERTER_H

#include "../Maze/InternalMouse.h"
#include "LogSystem.h"

class PathConverter {
 public:
  static std::string buildLFRPath(MazeNode* currNode,
                                  std::array<int, 2> currDir,
                                  std::vector<MazeNode*> nodePath);

 private:
  static std::string calculateMovesNeededBetweenHeadings(
      std::array<int, 2> from, std::array<int, 2> to);
  static std::string getMovesNeededBy4Cardinal(std::array<int, 2> from,
                                               std::array<int, 2> to);
  static int findIndexInVector(std::vector<std::array<int, 2>> vector,
                               std::array<int, 2> find);

  static MazeNode* currNode;
  static std::array<int, 2> currHeading;
};

#endif