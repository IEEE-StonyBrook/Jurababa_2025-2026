#ifndef INTERNALMOUSE_H
#define INTERNALMOUSE_H

#include <array>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "../Common/LogSystem.h"
#include "MazeGraph.h"

class InternalMouse
{
  public:
    InternalMouse(std::array<int, 2> startingRobotPosition, std::string startingRobotDirection,
                  std::vector<std::array<int, 2>> goalCells, MazeGraph* mazeGraph);

    void moveIMForwardOneCell(int cellNumberToMoveForward);
    void turnIM45DegreeStepsRight(int halfStepsRight);

    int  getMazeWidth();
    int  getMazeHeight();
    void setWallExistsLFR(char LFRdirection);
    void setWallExistsNESW(MazeNode* node, char NESWdirection);

    MazeNode*              getCurrentRobotNode();
    std::string            getCurrentRobotDirString();
    std::array<int, 2>     getCurrentRobotDirArray();
    MazeNode*              getNodeAtPos(int nodeX, int nodeY);
    std::vector<MazeNode*> getNodeNeighbors(MazeNode* node, bool includeDiagNeighbors = false);
    bool getCanMoveBetweenNodes(MazeNode* from, MazeNode* to, bool diagonalsAllowed = false);

    bool isAGoalCell(MazeNode* node);
    void resetSolverVariables();

    std::vector<std::array<int, 2>> getPossibleDirectionArrays();

  void setCurrentPosition(MazeNode* node);
  std::vector<std::array<int, 2>> getGoalCells();

  /**
   * @brief Calculates the Euclidean distance between two cells.
   *
   * @param cell1 The first cell.
   * @param cell2 The second cell.
   * @return The Euclidean distance.
   */
  static double euclideanDistance(MazeNode& cell1, MazeNode& cell2);

  /**
   * @brief Calculates the octile distance between two cells.
   *
   * @param cell1 The first cell.
   * @param cell2 The second cell.
   * @return The octile distance.
   */
  static double octileDistance(MazeNode& cell1, MazeNode& cell2);

  /**
   * @brief Returns the direction as a string based on a given direction offset.
   *
   * @param direction The direction offset as an array [dx, dy].
   * @return The direction string (n, ne, e, se, s, sw, w, nw).
   */
  std::string getDirectionAsString(const std::array<int, 2>& direction) const;

  /**
   * @brief Returns the direction to the left of the current mouse's direction.
   *
   * @return The direction string to the left (n, ne, e, se, s, sw, w, nw).
   */
  std::string getDirectionToTheLeft() const;

  /**
   * @brief Returns the direction to the right of the current mouse's direction.
   *
   * @return The direction string to the right (n, ne, e, se, s, sw, w, nw).
   */
  std::string getDirectionToTheRight() const;

  int findDirectionIndexInPossibleDirections(
      const std::string& direction) const;

 private:
  int indexOfDirection(std::string direction);
  std::string getNewDirectionAfterAddingHalfStepsRight(int halfStepsRight);

    std::string                     currentRobotDirection;
    std::array<int, 2>              currentRobotPosition;
    std::vector<std::array<int, 2>> goalCells;
    MazeGraph*                      mazeGraph;

    const std::array<std::string, 8>                possibleDirections = {"n", "ne", "e", "se",
                                                                          "s", "sw", "w", "nw"};
    const std::map<std::string, std::array<int, 2>> directionStringToOffsetArrayMap = {
        {"n", {0, 1}},  {"ne", {1, 1}},   {"e", {1, 0}},  {"se", {1, -1}},
        {"s", {0, -1}}, {"sw", {-1, -1}}, {"w", {-1, 0}}, {"n", {-1, 1}}};
};

#endif