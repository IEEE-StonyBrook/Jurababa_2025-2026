#include "../../Include/Maze/InternalMouse.h"

InternalMouse::InternalMouse(std::array<int, 2> startingRobotPosition,
                             std::string startingRobotDirection,
                             std::vector<std::array<int, 2>> goalCells,
                             MazeGraph* mazeGraph, LogSystem* logSystem)
    : currentRobotPosition(startingRobotPosition),
      currentRobotDirection(startingRobotDirection),
      goalCells(goalCells),
      mazeGraph(mazeGraph),
      logSystem(logSystem) {}

void InternalMouse::moveIMForwardOneCell(int cellNumberToMoveForward) {
  std::array<int, 2> directionOffsetToAdd =
      directionStringToOffsetArrayMap.at(currentRobotDirection);
  currentRobotPosition[0] += directionOffsetToAdd[0] * cellNumberToMoveForward;
  currentRobotPosition[1] += directionOffsetToAdd[1] * cellNumberToMoveForward;
}

void InternalMouse::turnIM45DegreeStepsRight(int halfStepsRight) {
  currentRobotDirection =
      getNewDirectionAfterAddingHalfStepsRight(halfStepsRight);
}

int InternalMouse::indexOfDirection(std::string direction) {
  for (int i = 0; i < possibleDirections.size(); i++) {
    if (possibleDirections[i] == direction) {
      return i;
    }
  }

  return -1;
}

std::string InternalMouse::getNewDirectionAfterAddingHalfStepsRight(
    int halfStepsRight) {
  if (halfStepsRight == 0) return currentRobotDirection;

  int currentDirectionIndex = indexOfDirection(currentRobotDirection);

  // Adding one to index = turning right half step in cardinal directions.
  int newDirectionIndex =
      ((currentDirectionIndex + halfStepsRight) % possibleDirections.size() +
       possibleDirections.size()) %
      possibleDirections.size();

  return possibleDirections[newDirectionIndex];
}

// bool InternalMouse::getDirNeededForNextNode(MazeNode* nextNode) {
//   MazeNode* currNode = getCurrentRobotNode();
//   std::array<int, 2> headingNeeded = {
//       nextNode->getCellXPos() - currNode->getCellYPos(),
//       nextNode->getCellYPos() - currNode->getCellYPos()};

//   if (abs(headingNeeded[0]) > 1 || abs(headingNeeded[1]) > 1) {
//   }
// }

void InternalMouse::setWallExistsLFR(char LFRdirection) {
  int halfStepsToAdd = 0;
  if (tolower(LFRdirection) == 'l') halfStepsToAdd = -2;
  if (tolower(LFRdirection) == 'r') halfStepsToAdd = 2;

  std::string directionStringToAddWall =
      getNewDirectionAfterAddingHalfStepsRight(halfStepsToAdd);
  std::array<int, 2> directionArrayToAddWall =
      directionStringToOffsetArrayMap.at(directionStringToAddWall);
  MazeNode* currentNode = getCurrentRobotNode();
  if (directionArrayToAddWall[1] == 1) currentNode->setWallInDirection('N');
  if (directionArrayToAddWall[0] == 1) currentNode->setWallInDirection('E');
  if (directionArrayToAddWall[1] == -1) currentNode->setWallInDirection('S');
  if (directionArrayToAddWall[0] == -1) currentNode->setWallInDirection('W');
}

MazeNode* InternalMouse::getNodeAtPos(int nodeX, int nodeY) {
  return mazeGraph->getNode(nodeX, nodeY);
}

MazeNode* InternalMouse::getCurrentRobotNode() {
  return mazeGraph->getNode(currentRobotPosition[0], currentRobotPosition[1]);
}

std::string InternalMouse::getCurrentRobotDirString() {
  return currentRobotDirection;
}

std::array<int, 2> InternalMouse::getCurrentRobotDirArray() {
  return directionStringToOffsetArrayMap.at(currentRobotDirection);
}

void InternalMouse::setWallExistsNESW(MazeNode* node, char NESWdirection) {
  MazeNode* currentNode = node;
  currentNode->setWallInDirection(NESWdirection);
}

std::vector<MazeNode*> InternalMouse::getNodeNeighbors(
    MazeNode* node, bool includeDiagNeighbors) {
  return mazeGraph->getNodeNeighbors(node, includeDiagNeighbors);
}

bool InternalMouse::getCanMoveBetweenNodes(MazeNode* from, MazeNode* to,
                                           bool diagonalsAllowed) {
  int diffX = to->getCellXPos() - from->getCellXPos();
  int diffY = to->getCellYPos() - from->getCellYPos();

  // Disallow diagonals if not allowed.
  if (diffX == 0 && diffY == 1) return !from->getIsWall('N');
  if (diffX == 1 && diffY == 0) return !from->getIsWall('E');
  if (diffX == 0 && diffY == -1) return !from->getIsWall('S');
  if (diffX == -1 && diffY == 0) return !from->getIsWall('W');

  // Check if diagonal movement is allowed.
  int fromXPos = from->getCellXPos(), fromYPos = from->getCellYPos();

  if (diagonalsAllowed) {
    if (diffX == 1 && diffY == 1)
      return !from->getIsWall('N') &&
             !mazeGraph->getNode(fromXPos, fromYPos + 1)->getIsWall('E');
    if (diffX == -1 && diffY == 1)
      return !from->getIsWall('N') &&
             !mazeGraph->getNode(fromXPos, fromYPos + 1)->getIsWall('W');
    if (diffX == 1 && diffY == -1)
      return !from->getIsWall('S') &&
             !mazeGraph->getNode(fromXPos, fromYPos - 1)->getIsWall('E');
    if (diffX == -1 && diffY == -1)
      return !from->getIsWall('S') &&
             !mazeGraph->getNode(fromXPos, fromYPos - 1)->getIsWall('W');
  }

  return false;
}

bool InternalMouse::isAGoalCell(MazeNode* node) {
  for (std::array<int, 2> goalCell : goalCells) {
    if (MazeNode::areNodesEqual(node,
                                mazeGraph->getNode(goalCell[0], goalCell[1]))) {
      return true;
    }
  }
  return false;
}

void InternalMouse::resetSolverVariables() {
  for (int col = 0; col < getMazeWidth(); col++) {
    for (int row = 0; row < getMazeHeight(); row++) {
      mazeGraph->getNode(col, row)->clearParentAndProcessed();
    }
  }
}

int InternalMouse::getMazeWidth() { return mazeGraph->getMazeWidth(); }

int InternalMouse::getMazeHeight() { return mazeGraph->getMazeHeight(); }

std::vector<std::array<int, 2>> InternalMouse::getPossibleDirectionArrays() {
  std::vector<std::array<int, 2>> directionArrays;
  for (std::pair<std::string, std::array<int, 2>> pair :
       directionStringToOffsetArrayMap) {
    directionArrays.push_back(pair.second);
  };

  return directionArrays;
}

void InternalMouse::setCurrentPosition(MazeNode* node) {
  currentRobotPosition = {node->getCellXPos(), node->getCellYPos()};
  // NOTE: this only updates position, not direction.
  // If you want to also adjust heading, you’ll need to add logic
  // based on previous node vs new node.
}

std::vector<std::array<int, 2>> InternalMouse::getGoalCells() {
  return goalCells;
}

double InternalMouse::euclideanDistance(MazeNode& cell1, MazeNode& cell2) {
  return std::sqrt(std::pow(cell1.getCellXPos() - cell2.getCellXPos(), 2) +
                   std::pow(cell1.getCellYPos() - cell2.getCellYPos(), 2));
}

// Static method to calculate octile distance
double InternalMouse::octileDistance(MazeNode& cell1, MazeNode& cell2) {
  int distanceX = std::abs(cell1.getCellXPos() - cell2.getCellXPos());
  int distanceY = std::abs(cell1.getCellYPos() - cell2.getCellYPos());
  return (distanceX + distanceY) +
         (std::sqrt(2) - 2) * std::min(distanceX, distanceY);
}

std::string InternalMouse::getDirectionAsString(
    const std::array<int, 2>& direction) const {
  const std::vector<std::string> possibleDirections = {"n", "ne", "e", "se",
                                                       "s", "sw", "w", "nw"};
  const std::vector<std::array<int, 2>>& possibleMouseDirections = {
      {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
  for (size_t i = 0; i < possibleMouseDirections.size(); ++i) {
    if (possibleMouseDirections[i][0] == direction[0] &&
        possibleMouseDirections[i][1] == direction[1]) {
      return possibleDirections[i];
    }
  }
  throw std::invalid_argument("Invalid direction offset.");
}

// Returns direction to the left
std::string InternalMouse::getDirectionToTheLeft() const {
  const std::vector<std::array<int, 2>>& possibleMouseDirections = {
      {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
  size_t currentIndex =
      findDirectionIndexInPossibleDirections(currentRobotDirection);
  size_t leftIndex =
      (currentIndex + 6) %
      possibleMouseDirections.size();  // Equivalent to turning left
  std::array<int, 2> newDirection = possibleMouseDirections[leftIndex];
  return getDirectionAsString(newDirection);
}

// Returns direction to the right
std::string InternalMouse::getDirectionToTheRight() const {
  const std::vector<std::array<int, 2>>& possibleMouseDirections = {
      {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
  size_t currentIndex =
      findDirectionIndexInPossibleDirections(currentRobotDirection);
  size_t rightIndex =
      (currentIndex + 2) %
      possibleMouseDirections.size();  // Equivalent to turning right
  std::array<int, 2> newDirection = possibleMouseDirections[rightIndex];
  return getDirectionAsString(newDirection);
}

// Finds the index of a given direction
int InternalMouse::findDirectionIndexInPossibleDirections(
    const std::string& direction) const {
  const auto& possibleMouseDirections =
      std::vector<std::string>{"n", "ne", "e", "se", "s", "sw", "w", "nw"};
  for (size_t i = 0; i < possibleMouseDirections.size(); i++) {
    if (possibleMouseDirections[i] == direction) {
      return i;
    }
  }
  throw std::invalid_argument(
      "Direction not listed as a possible mouse direction.");
}