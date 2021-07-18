// struct 
typedef struct next_station {
    int time;   //다음 역까지 가는데 걸리는 시간
    string station_name;   //다음역에 대한 station 정보

    next_station* next;   //다른 다음 역 정보
}next_station;

typedef struct station {
    string station_name;   //현재 역 이름
    next_station* next;   // 다음 역 리스트
    string line_number[10];   //현재 역 호선
} station;

// map
map<string, station*> stations_map;     // station 정보
map<string, string> user;       // user 정보      //sieun

// dijkstra
map<string, int> dijkstra_dist; //시작 역으로부터 모든 역까지의 최소거리
map<string, string*> dijkstra_path; //경로 저장할 map
map<string, int> dijkstra_path_index;   //경로 index 저장할 map
string dijkstra_stations[STATION_COUNT];    //역 이름 저장

struct compare {
    bool operator()(station*& s1, station*& s2) {
        return dijkstra_dist[s1->station_name] > dijkstra_dist[s2->station_name];
    }
};

// spfa
map<string, int> spfa_path_index;   // 경로 인덱스를 저장할 맵
map <string, string*> spfa_path;    // 경로를 저장할 맵
map<string, int> spfa_dist;         // 거리를 저장할 맵
// dfs
map<string, int>dfs_map;
string dfs_path;
map<string, int> dfs_dist;

int dijkstra_cmp;
int SPFA_cmp;
int DFS_cmp;
string path[721];
string n;
int cost = 99999;

// 함수
void make_station_node();   // station node 생성
void input_data();          // data 입력

int dijkstra(string start, string end);                     // dijkstra
string dijkstra_find_middle_station();                      // dijkstra middle station 찾기
string dijkstra_find_destination(string middle_station);    // dijkstra 최적 목적지 찾기
void dijkstra_print_result(string destination);             // dijkstra 결과 출력

int SPFA(string start, string end);                         // SPFA
string spfa_find_middle_station();                          // SPFA middle station 찾기
string SPFA_find_destination(string middle_station);        // SPFA 최적 목적지 찾기
void spfa_print_result(string destination);                 // SPFA 결과 출력

void start_DFS(string start, string end);                   // DFS
void DFS(string v, int sum, string s);                      // DFS 재귀
string dfs_find_middle_station();                           // DFS middle station 찾기
string DFS_find_destination(string middle_station);         // DFS 최적 목적지 찾기
void dfs_print_result(string destination);                  // DFS 결과 출력

