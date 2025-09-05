#include "PathConverter.h"

MazeNode* PathConverter::currNode = nullptr;
std::array<int, 2> PathConverter::currHeading = {0, 1};

std::string PathConverter::buildLFRPath(MazeNode* startNode,
                                        std::array<int, 2> startHeading,
                                        std::vector<MazeNode*> nodePath) {
  std::string lfrPath;
  if (nodePath.empty()) return lfrPath;

  currNode = startNode;
  currHeading = startHeading;
  for (MazeNode* nextNode : nodePath) {
    std::array<int, 2> headingNeeded = {
        nextNode->getCellXPos() - currNode->getCellXPos(),
        nextNode->getCellYPos() - currNode->getCellYPos()};
    std::string addedMoves =
        calculateMovesNeededBetweenHeadings(currHeading, headingNeeded);
    lfrPath += addedMoves;
    currNode = nextNode;
  }

  return lfrPath;
}

std::string PathConverter::calculateMovesNeededBetweenHeadings(
    std::array<int, 2> from, std::array<int, 2> to) {
  if (from[0] != 0 && from[1] != 0) {
    LOG_ERROR("PathConverter: Mouse on diagonal! Not allowed.");
    return "";
  }

  // N, E, S, W Directions
  if ((to[0] == 0 || to[1] == 0) && !(to[0] == 0 && to[1] == 0)) {
    return getMovesNeededBy4Cardinal(from, to);
    currHeading = to;
  }
  // Diagonal Directions
  else {
    std::array<int, 2> toVertical = {0, to[1]};
    std::array<int, 2> toHorizontal = {to[0], 0};
    std::string movesNeeded;
    movesNeeded += getMovesNeededBy4Cardinal(from, toVertical);
    movesNeeded += getMovesNeededBy4Cardinal(toVertical, toHorizontal);
    currHeading = toHorizontal;
  }
  return "";
}

std::string PathConverter::getMovesNeededBy4Cardinal(std::array<int, 2> from,
                                                     std::array<int, 2> to) {
  if (to[0] == 0 && to[1] == 0) return "";
  const std::vector<std::array<int, 2>> directionsArray = {
      {0, 1}, {1, 0}, {0, -1}, {-1, 0}};

  int fromIndex = findIndexInVector(directionsArray, from);
  int toIndex = findIndexInVector(directionsArray, to);
  if (fromIndex == -1 | toIndex == -1) {
    LOG_ERROR("PathConverter: Not 4 cardinal directions!");
    return "";
  }
  int rightTurnsNeeded = toIndex - fromIndex;
  if (rightTurnsNeeded == 3)
    return "L#F#";
  else if (rightTurnsNeeded == 2)
    return "R#R#F#";
  else if (rightTurnsNeeded == 1)
    return "R#F#";
  else if (rightTurnsNeeded == 0)
    return "F#";

  return "";
}

int PathConverter::findIndexInVector(std::vector<std::array<int, 2>> vector,
                                     std::array<int, 2> find) {
  int counter = 0;
  for (std::array<int, 2> array : vector) {
    if (array[0] == find[0] && array[1] == find[1]) return counter;
    counter++;
  }
  return -1;
}