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
      break;

    case 'e':
      addThereIsWall('e');
      if (ECell != nullptr) ECell->addThereIsWall('w');
      break;

    case 's':
      addThereIsWall('s');
      if (SCell != nullptr) SCell->addThereIsWall('n');
      break;

    case 'w':
      addThereIsWall('w');
      if (WCell != nullptr) WCell->addThereIsWall('e');
      break;
  }
}

bool MazeNode::getIsWall(char NESWdirection) {
  switch (tolower(NESWdirection)) {
    case 'n':
      return isThereNWall;
      break;
    case 'e':
      return isThereEWall;
      break;
    case 's':
      return isThereSWall;
      break;
    case 'w':
      return isThereWWall;
      break;
  }

  return false;
}

void MazeNode::addThereIsWall(char NESWdirection) {
  switch (tolower(NESWdirection)) {
    case 'n':
      isThereNWall = true;
      break;
    case 'e':
      isThereEWall = true;
      break;
    case 's':
      isThereSWall = true;
      break;
    case 'w':
      isThereWWall = true;
      break;
  }
}

void MazeNode::setCellInDirection(MazeNode* node, char NESWdirection) {
  switch (tolower(NESWdirection)) {
    case 'n':
      NCell = node;
      break;
    case 'e':
      ECell = node;
      break;
    case 's':
      SCell = node;
      break;
    case 'w':
      WCell = node;
      break;
  }
}

void MazeNode::clearParentAndProcessed() {
  parentNode= nullptr;
  isProcessed = false;
}

bool MazeNode::areNodesEqual(MazeNode* n1, MazeNode* n2) {
  return n1->getCellXPos() == n2->getCellXPos() and
         n1->getCellYPos() == n2->getCellYPos();
}

int MazeNode::getCellXPos() { return mazeXPos; }
int MazeNode::getCellYPos() { return mazeYPos; }

bool MazeNode::getCellIsExplored() { return cellIsExplored; }
void MazeNode::markAsExplored() { cellIsExplored = true; }