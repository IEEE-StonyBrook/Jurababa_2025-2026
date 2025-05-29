#include "MazeNode.h"

#include <cctype>

MazeNode::MazeNode(int mazeXPos, int mazeYPos)
    : mazeXPos(mazeXPos), mazeYPos(mazeYPos) {}

void MazeNode::setWallInDirection(char NESWdirection) {
  switch (tolower(NESWdirection)) {
    // Adds shared walls between two cells.
    case 'n':
      addThereIsWall('n');
      if (NCell != nullptr) NCell->addThereIsWall('s');

    case 'e':
      addThereIsWall('e');
      if (ECell != nullptr) ECell->addThereIsWall('w');

    case 's':
      addThereIsWall('s');
      if (SCell != nullptr) SCell->addThereIsWall('n');

    case 'w':
      addThereIsWall('w');
      if (WCell != nullptr) WCell->addThereIsWall('e');
  }
}

bool MazeNode::getIsWall(char NESWdirection) {
  switch (tolower(NESWdirection)) {
    case 'n':
      return isThereNWall;
    case 'e':
      return isThereEWall;
    case 's':
      return isThereSWall;
    case 'w':
      return isThereWWall;
  }

  return false;
}

void MazeNode::addThereIsWall(char NESWdirection) {
  switch (tolower(NESWdirection)) {
    case 'n':
      isThereNWall = true;
    case 'e':
      isThereEWall = true;
    case 's':
      isThereSWall = true;
    case 'w':
      isThereWWall = true;
  }
}

void MazeNode::setCellInDirection(MazeNode* node, char NESWdirection) {
  switch (tolower(NESWdirection)) {
    case 'n':
      NCell = node;
    case 'e':
      ECell = node;
    case 's':
      SCell = node;
    case 'w':
      WCell = node;
  }
}

void MazeNode::clearParentAndProcessed() {
  parentNode= nullptr;
  isProcessed = false;
}

static bool areNodesEqual(MazeNode* n1, MazeNode* n2) {
  return n1->getCellXPos() == n2->getCellXPos() and
         n1->getCellYPos() == n2->getCellYPos();
}

int MazeNode::getCellXPos() { return mazeXPos; }
int MazeNode::getCellYPos() { return mazeYPos; }

bool MazeNode::getCellIsExplored() { return cellIsExplored; }
void MazeNode::markAsExplored() { cellIsExplored = true; }