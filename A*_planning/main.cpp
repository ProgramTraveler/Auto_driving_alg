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
    int count = 1; // 统计 new 的次数
    int count_del = 0; // 统计 delete 的次数

    // 进入循环 开始搜索
    // 搜索到终点则返回路径
    vector<Point> path;
    while (!openlist.empty()) {
        // 获取当前搜索的节点 current 既 openlist 中 f 最小的节点
        sort(openlist.begin(), openlist.end(), NodeCompare{}); // 排序 自定义排序结果
        Node *current = *openlist.begin(); // 获取 f 最小的点

        // 将 current 对应的点从 opnelist 中删除
        openlist.erase(openlist.begin());

        // 将 current 添加到 closelist 中
        closelist.push_back(current);

        // 对当前搜索节点 current 进行分类讨论 -> 其实就对四个方向进行讨论
        /*
            1. current 是终点 则直接返回 path
            2. current 不是终点 需要讨论其邻近节点 neighbors
        */

        if (current->point.x == goal.x && current->point.y == goal.y) {
            while (current != nullptr) { // 利用父节点 从终点向起点回溯最短路径
                /*
                    因为起点没有父节点
                    所以起点的时候 current 的父节点为空
                */
                path.push_back(current->point);

                current = current->parent;
            }

            reverse(path.begin(), path.end()); // 翻转路径
            // openlist 和 closelist 存储的元素都是 new 动态分配的内存 需要delete 否则会造成内存泄漏
            for (auto o : openlist) delete o, count_del ++;
            for (auto c : closelist) delete c, count_del ++;
            cout << "new count is " << count << " " << "delete count is " << count_del << endl;
            return path;
        }

        int x = current->point.x;
        int y = current->point.y;

        vector<Point> neighbors = { // 八个方位
            {x - 1, y - 1}, {x - 1, y}, {x + 1, y},
            {x, y - 1},   {x, y + 1},
            {x + 1, y - 1},{x + 1, y},{x + 1, y + 1}
        };

        // 遍历所有方位的邻近节点
        for (auto n : neighbors) {
            if ((n.x >= 0 && n.x < row) && (n.y >= 0 && n.y < col) && gridmap[n.x][n.y] == 0) { // 边界和障碍物判断
                // n 在 closelist 中 表示已经探索过 直接跳过
                bool inclose = false;

                for (auto c : closelist) {
                    if (c->point.x == n.x && c->point.y == n.y) {
                        inclose = true;

                        break;
                    }
                }

                if (inclose) continue;

                // n 是否在 openlist 中
                bool inopen = false;
                for (auto o : openlist) {
                    if (o->point.x == n.x && o->point.y == n.y) {
                        inopen = true;

                        // 对比 f 值 更新代价和父节点 parent
                        double g = current->g + n.distance(current->point); // g -> 邻近节点 n 到起点的距离 = current 到起点的距离 + current 到邻近节点 n 的距离
                        double h = n.distance(goal); // 邻近节点到终点的估计代价距离
                        double f = g + h;

                        if (f < (o->f)) {
                            o->f = f;
                            o->parent = current;
                        }

                        break;
                    }
                }

                if (inopen) continue;

                // 不在 openlist 和 closelist 中
                // 计算代价值 添加到 openlist 中
                double g = current->g + n.distance(current->point); // g -> 邻近节点 n 到起点的距离 = current 到起点的距离 + current 到邻近节点 n 的距离
                double h = n.distance(goal); // 邻近节点到终点的估计代价距离
                double f = g + h;
                openlist.push_back(new Node(n, g, h, current));
                count ++;

            } 
        }
    }

    // 搜索完成没有路径 表示规划失败 返回空路径
    for (auto o : openlist) delete o, count_del ++;
    for (auto c : closelist) delete c, count_del ++;
    cout << "new count is " << count << " " << "delete count is " << count_del << endl;
    return path;
}

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

    cout << "the point count is " << path.size() << endl;

    for (auto p : path) cout << "(" << p.x << ", " << p.y << ")" << "-> ";
    cout << endl;

    return 0;
}