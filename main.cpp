//
//  main.cpp
//  zum_du1
//
//  Created by Kirill on 22.03.2023.
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <unordered_set>
#include <queue>
#include <list>
#include <filesystem>
#include <stack>
#include <set>
#include <time.h>
namespace fs = std::filesystem;
using namespace std;


class PathFinder
{
private:
    struct Ceil {
        int x;
        int y;
        Ceil(){}
        Ceil(int x, int y): x(x), y(y) {}
        Ceil operator+(const Ceil & other) const{
            Ceil copy = *this;
            copy.x += other.x;
            copy.y += other.y;
            return copy;
        }
        bool isValid(int length, int height) {
            return x >= 0 && y >= 0 && x < length && y < height;
        }
        bool operator==(const Ceil & other) const{
            return x == other.x && y==other.y;
        }
        bool operator<(const Ceil & other) const{
            return x < other.x || y < other.y;
        }
    };
    struct CeilHash {
      size_t operator() (const Ceil & ceil) const {
        return hash<int>{}(ceil.x) ^ hash<int>{}(ceil.y);
      }
    };
    
    
public:
    PathFinder(const string & filePath):
    height(0),
    length(0),
    expandedCount(0)
    {
        loadMap(filePath);
    }
    
    void doDfs() {
        expandedCount = 0;
        unordered_set<Ceil, CeilHash> visited;
        unordered_map<Ceil, Ceil, CeilHash> parents;
        stack<Ceil> nodeStack;
        nodeStack.push(start);
        visited.insert(start);
        while (!nodeStack.empty()) {
            Ceil currentNode = nodeStack.top();
            history.push_back({currentNode});
            expandedCount++;
            nodeStack.pop();
            if (currentNode == end) {
                break;
            }
            for (const auto & neigh : graphRepresentation[currentNode]) {
                if (visited.count(neigh) == 0) {
                    nodeStack.push(neigh);
                    visited.insert(neigh);
                    parents[neigh] = currentNode;
                }
            }
        }
        
        buildPath(parents);
    }
   
    void doGreedy() {
        unordered_map<Ceil, double, CeilHash> distantions = getDistantions();
        unordered_set<Ceil, CeilHash> visited;
        unordered_map<Ceil, Ceil, CeilHash> parents;
        auto comp = [](const pair<Ceil, double> & a1, const pair<Ceil, double> & a2){return a1.second > a2.second;};
        vector<pair<Ceil, double>> nodeHeap;
        nodeHeap.push_back({start, distantions[start]});
        history.push_back({start});
        visited.insert(start);
        
        while (!nodeHeap.empty()) {
            pop_heap(nodeHeap.begin(), nodeHeap.end(), comp);
            Ceil current = nodeHeap.back().first;
            nodeHeap.pop_back();
            expandedCount++;
            history.push_back({current});
            if (current == end) {
                break;
            }
            for (const auto & neigh : graphRepresentation[current]) {
                if (visited.count(neigh) == 0) {
                    nodeHeap.push_back({neigh, distantions[neigh]});
                    push_heap(nodeHeap.begin(), nodeHeap.end(), comp);
                    parents[neigh] = current;
                    visited.insert(neigh);
                }
            }
        }
        buildPath(parents);
        
    }
    struct cmpForDist{
            static constexpr double THRESHOLD = 1e-18;

            bool operator()(const pair<Ceil, double> & a1, const pair<Ceil, double> & a2 )const {
                if (abs(a1.second - a2.second) <= THRESHOLD) { // use threshold for comparison
                    return a1.first < a2.first;
                }
                return a1.second < a2.second;
            }
        };
    
    void doAStar() {
        unordered_map<Ceil, Ceil, CeilHash> parents;
        unordered_map<Ceil, double, CeilHash> distFromEnd = getDistantions();
        unordered_map<Ceil, int, CeilHash> distFromStart;
        set<pair<Ceil, double>, cmpForDist> nodeQueue;
        

        distFromStart[start] = 0;

        nodeQueue.insert({start, distFromEnd[start]});
        while (!nodeQueue.empty()) {
            //pop_heap(nodeHeap.begin(), nodeHeap.end(), comp);
            Ceil current = nodeQueue.begin()->first;
            nodeQueue.erase(nodeQueue.begin());
            history.push_back({current});
            expandedCount++;
            if (current == end) {
                break;
            }
            
            int dist = distFromStart[current] + 1;
            for (const auto & neigh : graphRepresentation[current]) {
                if (distFromStart.count(neigh) == 0 || distFromStart[neigh] > dist) {
                    
                    auto iter = nodeQueue.find({neigh, distFromStart[neigh] + distFromEnd[neigh]});
                    
                    if (iter != nodeQueue.end()) {
                        nodeQueue.erase(iter);
                    }
                 
                    nodeQueue.insert(make_pair(neigh, dist + distFromEnd[neigh]));
                    distFromStart[neigh] = dist;
                    parents[neigh] = current;
                }
            }
            
        }
        buildPath(parents);
        
    }
    void doRandom() {
        srand(time(NULL));
        vector<pair<Ceil, int>> randomHeap;
        unordered_set<Ceil, CeilHash> visited;
        unordered_map<Ceil, Ceil, CeilHash> parents;
        auto comp = [](pair<Ceil, int> & a1, pair<Ceil, int> & a2) {return a1.second > a2.second;};
        
        randomHeap.push_back({start, rand() % 100000});
        visited.insert(start);
        
        while (!randomHeap.empty()) {
            pop_heap(randomHeap.begin(), randomHeap.end(), comp);
            Ceil currentNode = randomHeap.back().first;
            randomHeap.pop_back();
            if (currentNode == end) {
                break;
            }
            history.push_back({currentNode});
            for (const auto & neigh : graphRepresentation[currentNode]) {
                if (visited.count(neigh) == 0) {
                    visited.insert(neigh);
                    randomHeap.push_back({neigh, rand() % 100000});
                    push_heap(randomHeap.begin(), randomHeap.end(), comp);
                    parents[neigh] = currentNode;
                    
                }
            }
            
        }
        
        expandedCount = (int) visited.size();
        buildPath(parents);
        
    }
    void doBfs() {
        queue<Ceil> nodeQueue;
        unordered_set<Ceil, CeilHash> visited;
        unordered_map<Ceil, Ceil, CeilHash> parents;
        nodeQueue.push(start);
        visited.insert(start);
        history.push_back({start});
        while (!nodeQueue.empty()) {
            Ceil currentNode = nodeQueue.front();
            nodeQueue.pop();
            if (currentNode == end) {
                break;
            }
            
            vector<Ceil> v;
            for (const auto & neigh : graphRepresentation[currentNode]) {
                if (visited.count(neigh) == 0) {
                    visited.insert(neigh);
                    nodeQueue.push(neigh);
                    parents[neigh] = currentNode;
                    v.push_back(neigh);
                    
                }
            }
            if (!v.empty()) {
                history.push_back(v);
            }
            
        }
        expandedCount = (int) visited.size();
        buildPath(parents);
        
    }
    void printPath() {
        
        
        if (path.empty()) {
            cout << "Cesta nenalezena.\n" << "Odkrokovani:" << endl;
        }
        else {
            cout << "Cesta nalezena!" << endl;
            cout << "Odkrokovani: " << endl;
        }
        unordered_set<Ceil, CeilHash> path_copy (path.begin(), path.end());
        bool skip = false;
        int user;
        auto map_copy = textMap;
        for (int i = 0; i < history.size(); i++) {
            for (const auto & ceil : history[i]) {
                map_copy[ceil.y][ceil.x] = '#';
                if (path_copy.count(ceil)) {
                    map_copy[ceil.y][ceil.x] = 'P';
                }
            }
            
            if (!skip) {
                cout << "1. Pristi krok\n" << "2. Ukazat najednou" << endl;
                cin >> user;
                if (user == 2) {
                    skip = true;
                }
                
                printMap(map_copy);
                
            }
            map_copy[start.y][start.x] = 'S';
            map_copy[end.y][end.x] = 'E';
            
        }
        printMap(map_copy);
        if (!path.empty()) {
            cout << "Delka cesty: " << path.size() << endl;
        }
        cout << "Uzlu otevreno: " << expandedCount << endl;
        
    }
   
private:
    unordered_map<Ceil, double, CeilHash> getDistantions() {
        unordered_map<Ceil, double, CeilHash> dists;
        for (auto & c : graphRepresentation) {
            dists[c.first] = sqrt(pow(c.first.x - end.x, 2) + pow(c.first.y - end.y, 2));
        }
        return dists;
    }
    void buildPath(unordered_map<Ceil, Ceil, CeilHash> & parents) {
        Ceil c = end;
        while (parents.count(c)) {
            path.push_front(parents[c]);
            c = parents[c];
        }
        path.push_back(start);
    }
    void printMap(const vector<string> & map) {
        cout << "------------------------------------" << endl;
        
        for (const auto & str : map) {
            cout << str << endl;
        }
        cout << "------------------------------------" << endl;

    }
    
    void loadMap(const string & filePath) {
        ifstream ifile (filePath);
        string buffer;
        getline(ifile, buffer);
        length = (int) buffer.length();
        textMap.push_back(buffer);
        height = 1;
        while (getline(ifile, buffer)) {
            
            if (buffer[0] == 'X') {
                if (buffer.length() != length) {
                    throw invalid_argument("Invalid map!");
                }
                height++;
                textMap.push_back(buffer);
            }
            else if (buffer[0] == 's') {
                stringstream stream (buffer);
                string tmp;
                stream >> tmp >> start.x >> tmp >> start.y;
            }
            else if (buffer[0] == 'e') {
                stringstream stream (buffer);
                string tmp;
                stream >> tmp >> end.x >> tmp >> end.y;
            }
        }
        buildGraph();
    }
    
    void buildGraph() {
        vector<Ceil> neighs = {{0, 1}, {1, 0}, {-1, 0}, {0, -1}};
        for (int i = 0; i < height; i++) {
            for (int k = 0; k < length; k++) {
                if (textMap[i][k] == ' ') {
                    Ceil current (k, i);
                    for (const auto & neigh: neighs) {
                        Ceil n = (current + neigh);
                        if (n.isValid(length, height) && textMap[n.y][n.x] == ' ') {
                            graphRepresentation[current].push_back(n);
                        }
                    }
                }
            }
        }
    }
    
    
    unordered_map<Ceil, vector<Ceil>, CeilHash> graphRepresentation;
    vector<string> textMap;
    Ceil start;
    Ceil end;
    list<Ceil> path;
    vector<vector<Ceil>> history;
    int height;
    int length;
    int expandedCount;
};
#define START_ST  0
#define FILE_CHOOSE_ST 1
#define ALG_CHOOSE_ST 2

int main(int argc, const char * argv[]) {

    int state = START_ST;
    string path = "./testovaci_data";
    vector<fs::directory_entry> files;
    for (const auto & entry : fs::directory_iterator(path))
        files.push_back(entry);
    
    int user = 0;
    string filePath;

    while (true) {
        if (state == START_ST) {
            cout << "Zvolte mapu: " << endl;
            for (int i = 0; i < files.size(); i++) {
                cout << i << ".: " << files[i].path().filename() << endl;
            }
            state = FILE_CHOOSE_ST;
            continue;
        }
        cin >> user;
        if (state == FILE_CHOOSE_ST) {
            filePath = files[user].path();
            state = ALG_CHOOSE_ST;
            cout << "Vyberte algoritmus hledani nejkratsi cesty:\n" <<
            "1. BFS\n" << "2. DFS\n" << "3. A*\n" << "4. Random\n" << "5. Greedy" << endl;
        }
        else if (state == ALG_CHOOSE_ST) {
            PathFinder p (filePath);
            if (user == 1) {
                p.doBfs();
                p.printPath();
                state = START_ST;
            }
            else if (user == 2) {
                p.doDfs();
                p.printPath();
                state = START_ST;
            }
            else if (user == 3) {
                p.doAStar();
                p.printPath();
                state = START_ST;
            }
            else if (user == 4 ) {
                p.doRandom();
                p.printPath();
                state = START_ST;
            }
            else if (user == 5) {
                p.doGreedy();
                p.printPath();
                state = START_ST;
            }
            else {
                cout << "Nespravne cislo" << endl;
            }
            
            
        }
    }

    return 0;
}


