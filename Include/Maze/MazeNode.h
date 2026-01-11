#ifndef MAZENODE_H
#define MAZENODE_H

class MazeNode
{
  public:
    MazeNode(int mazeXPos, int mazeYPos);

    int  getCellXPos();
    int  getCellYPos();
    bool getCellIsExplored();
    bool getIsWall(char NESWdirection);

    /**
     * @brief Counts the number of known walls around this cell.
     * @return Number of walls (0-4).
     */
    int getWallCount();

    void markAsExplored();
    void setCellInDirection(MazeNode* node, char NESWdirection);
    void setWallInDirection(char NESWdirection);

    static bool areNodesEqual(MazeNode* n1, MazeNode* n2);

    MazeNode* parentNode;
    bool      isProcessed;
    void      clearParentAndProcessed();

  private:
    void addThereIsWall(char NESWdirection);

    int       mazeXPos, mazeYPos;
    MazeNode* NCell;
    MazeNode* ECell;
    MazeNode* SCell;
    MazeNode* WCell;
    bool      isThereNWall, isThereEWall, isThereSWall, isThereWWall;
    bool      cellIsExplored;
};

#endif