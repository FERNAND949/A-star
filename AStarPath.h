#include "node.h"
#include <fstream>
#include <string>
#include <sstream>
#include <vector>


class AStarPath{

public:
	AStarPath(){}
	virtual ~AStarPath(){}

	void Init();

	// A*による大域的な検索
	void PathPlan();

private:

	Node s,g;// スタートとゴール
	Node m,n;  // A*で使用

	std::vector<Position> obst;// 障害物の座標
	std::vector<Node> astarData;//A*による大域的な経路 位置と親ノードと自ノードの番号のみ使用
};

















