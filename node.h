/*!
 * @file nodeを記述するためのクラス
 */
#include "position.h"

class Node{

public:
	/*
	 * @brief コンストラクタ
	 */
	Node(){
		mour_num = 0;
		mpre_num = 0;
	}

	/*
	 * @brief デストラクタ
	 */
	virtual~Node(){}

	/*
	 * @brief 位置の設定
	 */
	void SetPos(double x, double y){
		mpos.SetPosition(x,y);
	}

	/*
	 * @brief 位置の取得
	 * @return 位置
	 */
	Position OurPos() const { return mpos; }


	/*
	 * @brief 前のnode番号の設定
	 * @param[in] pre 前のノード番号
	 */
	void SetPreNum(unsigned int pre) { mpre_num = pre; }
	unsigned int PreNum()  { return mpre_num; }

	/*
	 * @brief このノードの番号を設定
	 * @param[in] num ノード番号
	 */
	void SetOurNum(unsigned int num) { mour_num = num; } 

	/*
	 * @brief このノードの番号を戻す
	 * @return ノード番号
	 */
	unsigned int OurNum()  { return mour_num; }

	//コスト
	double Cost() { return mcost; }

	//ゴールまでの最小推定値
	double calculateGoalCost(double gx, double gy){

		double x = gx-mpos.x();
		double y = gy-mpos.y();

		mcost = sqrt(x*x + y*y);
		return mcost;
	}


private:

	Position mpos;		//!
	unsigned int mour_num;	//! ノード番号
	unsigned int mpre_num;	//! 一つ前のノード番号
	double mcost;           //! コスト ヒューリスティックなどを加えた最終的なコスト
};



















