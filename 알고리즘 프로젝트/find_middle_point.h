// struct 
typedef struct next_station {
    int time;   //���� ������ ���µ� �ɸ��� �ð�
    string station_name;   //�������� ���� station ����

    next_station* next;   //�ٸ� ���� �� ����
}next_station;

typedef struct station {
    string station_name;   //���� �� �̸�
    next_station* next;   // ���� �� ����Ʈ
    string line_number[10];   //���� �� ȣ��
} station;

// map
map<string, station*> stations_map;     // station ����
map<string, string> user;       // user ����      //sieun

// dijkstra
map<string, int> dijkstra_dist; //���� �����κ��� ��� �������� �ּҰŸ�
map<string, string*> dijkstra_path; //��� ������ map
map<string, int> dijkstra_path_index;   //��� index ������ map
string dijkstra_stations[STATION_COUNT];    //�� �̸� ����

struct compare {
    bool operator()(station*& s1, station*& s2) {
        return dijkstra_dist[s1->station_name] > dijkstra_dist[s2->station_name];
    }
};

// spfa
map<string, int> spfa_path_index;   // ��� �ε����� ������ ��
map <string, string*> spfa_path;    // ��θ� ������ ��
map<string, int> spfa_dist;         // �Ÿ��� ������ ��
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

// �Լ�
void make_station_node();   // station node ����
void input_data();          // data �Է�

int dijkstra(string start, string end);                     // dijkstra
string dijkstra_find_middle_station();                      // dijkstra middle station ã��
string dijkstra_find_destination(string middle_station);    // dijkstra ���� ������ ã��
void dijkstra_print_result(string destination);             // dijkstra ��� ���

int SPFA(string start, string end);                         // SPFA
string spfa_find_middle_station();                          // SPFA middle station ã��
string SPFA_find_destination(string middle_station);        // SPFA ���� ������ ã��
void spfa_print_result(string destination);                 // SPFA ��� ���

void start_DFS(string start, string end);                   // DFS
void DFS(string v, int sum, string s);                      // DFS ���
string dfs_find_middle_station();                           // DFS middle station ã��
string DFS_find_destination(string middle_station);         // DFS ���� ������ ã��
void dfs_print_result(string destination);                  // DFS ��� ���

