#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

// 定义地图上的点
struct Point {
    int x, y; // 栅格行列
    Point (int x, int y) : x(x), y(y) {}; // 参数列表初始化
    double distance(Point &p) { // 计算两个点之间的欧式及距离
        return sqrt((x - p.x)^2 + (y - p.y)^2); 
    };
};

// 定义节点
struct Node {
    Point point; // 栅格点
    double g, h, f; // 代价值 f -> 总代价 g -> 到起点的代价值 h -> 到终点的代价值 启发式函数
    Node *parent; // 父节点
    Node (Point point, double g, double h, Node *parent = nullptr) : point (point), g(g), h(h), f(g + h), parent(parent) {};
};

// 自定义 Node* 排序规则
struct NodeCompare {
    bool operator () (Node *n1, Node *n2) {
        return n1->f < n2->f; // 升序排列
    };
};

// 基于栅格地图的路径规划算法 A*
// 返回由 Point 组成的路径 path
// 输入为地图 起点 终点 
vector<Point> AstarPathPlanning (vector<vector<int>> &gridmap, Point &start, Point &goal) {
    // 获取地图参数
    int row = gridmap.size(); // 行 -> 地图的宽
    int col = gridmap[0].size(); // 列 -> 地图的长

    // 定义 openlist closelist
    vector<Node *> openlist; // 待搜索的节点
    vector<Node *> closelist; // 已搜索的节点

    openlist.push_back(new Node(start, start.distance(start), start.distance(goal))); // 初始化 将起点添加到 openlist

    // 进入循环 开始搜索
    // 搜索到终点则返回路径
    vector<Point> path;
    while (!openlist.empty()) {
        // 获取当前搜索的节点 current 既 openlist 中 f 最小的节点
        sort(openlist.begin(), openlist.end()); // 排序 自定义排序结果


        

    };

    // 搜索完成没有路径 表示规划失败 返回空路径
    return path;
};

int main () {
    // 定义地图
    vector <vector<int>> gridmap = {
        {0, 0, 1, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };

    // 定义起点和终点
    Point start{0, 0};
    Point goal{4, 4};

    vector<Point> path = AstarPathPlanning(gridmap, start, goal);


    return 0;
}