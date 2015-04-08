#include "AStarPath.h"
#include <stdlib.h>
#include <algorithm>


/**************************************
 * 単位はmmで設定
***************************************/



void AStarPath::Init(){	

	std::cout << "Init Func" << std::endl;

	s.SetPos(0.0, 0.0);// スタートの位置
	s.SetOurNum(1);    // ノード番号
	s.SetPreNum(0);    // 親ノードの番号

	g.SetPos(1000.0, 1000.0);// ゴールの位置
	g.SetOurNum(g.OurPos().x() + g.OurPos().y()*10);// ノード番号の初期化
}

void AStarPath::PathPlan(){

	int w,h,wdiv,hdiv;     //グリッドマップの大きさと分割する大きさ
	float heuristic;

	int openListNo;
	int nodeNo;
	float minCost;        // 最小コスト
	int mCost;            // ノードのコスト
	int mx,my;            // 移動するノードの座標

	float fm;             // 全体のコスト
	float ga;             // 基準となるノードまでのコスト
	float costNM;         // 基準のノードから隣接ノードまでのコスト
	int oflag,cflag,flag;

	minCost = 0;
	openListNo = -1;
	flag=nodeNo=1;

	//A*に必要なリスト
	std::vector<Node> openList;
	std::vector<Node> closeList;

	openList.clear();
	closeList.clear();
	openList  = std::vector<Node>();
	closeList = std::vector<Node>();

	astarData.clear();

	//グリッドマップの大きさ
	w = 1000;	    // x方向
	h = 1000;	    // y方向

	wdiv = 100;	    // x方向をどのくらいの間隔で区切るか
	hdiv = 100;	    // y方向をどのくらいの間隔で区切るか

	minCost = w*h;      // 比較するコストの初期化

	heuristic = sqrt(pow(g.OurPos().x()-s.OurPos().x(),2) + pow(g.OurPos().y()-s.OurPos().y(),2));// ヒューリスティック ゴールまでの距離
	openList.push_back(s);// openListにスタートノードを入れる

	std::cout << "A-star start" << std::endl;
	while(flag){

		if(openList.size()==0) break;//ゴールまでの経路が存在しない
		//今は隣接している8つのノードに移動するコストはすべて同じとするので移動コストはヒューリスティックのみ
		for(uint i=0; i<openList.size(); i++){
			
			float opy = openList[i].OurPos().y();
			float opx = openList[i].OurPos().x();

			//コストの比較
			heuristic = sqrt(pow(g.OurPos().x()-opx,2) + pow(g.OurPos().y()-opy,2));

			if(heuristic < minCost){
				openListNo = i;
				minCost    = heuristic;
				m          = openList[i];
			}
		}

		if((m.OurPos().x()==g.OurPos().x()) && (m.OurPos().y()==g.OurPos().y())){//探索終了
			closeList.push_back(m);
			nodeNo = closeList.size()-1;
			flag = 0;
			break;
		}
		else{
			closeList.push_back(m);
			std::vector<Node>::iterator open;
			open = openList.erase(openList.begin() + openListNo);
		}


		//隣接ノードについて
		for(int y=-hdiv; y<(hdiv+1); y+=hdiv){// 分割数
			for(int x=-wdiv; x<(wdiv+1); x+=wdiv){// 分割数
			
				mx = m.OurPos().x() + x;
				my = m.OurPos().y() + y;
				
				// xyの範囲の指定，今は指定した範囲の±の値 
				// ここで障害物のデータとの照合が必要な場合はする
				if(mx<-w) continue;
				else if(my<-h) continue;
				else if(mx>w) continue;
				else if(my>h) continue;
				else if(y==0 && x==0) continue;// 求めたノードが今と同じ位置

			
				// 以下A*の処理
				else{
					fm = sqrt(pow(g.OurPos().x()-mx,2) + pow(g.OurPos().y()-my,2));
					oflag=cflag=0;

					// openListとcloseListのどっちに入っているか検索
					for(uint i=0; i<openList.size(); i++){
						if((mx==openList[i].OurPos().x()) && (my==openList[i].OurPos().y())){
							oflag=1;
							openListNo=i;
							break;
						}
					}
					for(uint i=0; i<closeList.size(); i++){
						if((mx==closeList[i].OurPos().x()) && (my==closeList[i].OurPos().y())){
							cflag=1;
							openListNo=i;
							break;
						}
					}
					// openListにある場合
					if(oflag){
						if(fm < heuristic){
							openList[openListNo].SetPreNum(m.OurNum());
						}
					}
					//closeListにある場合
					else if(cflag){
						if(fm  < heuristic){
						
							n = closeList[openListNo];
							n.SetPreNum(m.OurNum());
							openList.push_back(n);
							std::vector<Node>::iterator close;
							close = closeList.erase(closeList.begin() + openListNo);
						}
					}
					//どっちにも無い場合
					else{
						nodeNo++;
						n.SetPos(mx,my);
						n.SetOurNum(nodeNo);
						n.SetPreNum(m.OurNum());
						openList.push_back(n);
					}
				}
			}
		}
	}

	astarData.push_back(closeList[nodeNo]);
	std::vector<Node>::iterator pathIte = astarData.begin();

		
	flag=1;
	while(flag){
		for(uint n=0; n<closeList.size(); n++){
			if(closeList[nodeNo].PreNum()==closeList[n].OurNum()){
				nodeNo=n;// 親ノードをたどる
				break;
			}
		}
	
		if( closeList[nodeNo].PreNum() ==  0 ) flag=0;// スタート位置の場合抜ける

		astarData.insert(pathIte, 1, closeList[nodeNo]);
		pathIte = astarData.begin();
	}

	std::ofstream ofs;
	ofs.open("A-star.csv",std::ios::trunc);// 経路の出力先

	for(uint i=0; i<astarData.size(); i++){
		
		ofs << astarData[i].OurPos().x() << "," << astarData[i].OurPos().y() << std::endl;
	}


	std::cout << "A-star end" << std::endl;
	std::cout << std::endl;
}



