#pragma warning(disable:4996)
#include <locale>
#include <codecvt>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <queue>
#include "json/json.h"

#include <time.h>

#define TRANSFER_COST 10
#define MAX 10000
#define STATION_COUNT 721

using namespace std;
#include "find_middle_point.h"

int main() {
    // input
    input_data();

    // 모든 station을 node로 생성
    make_station_node();

    // dijkstra & spfa 동적할당
    for (auto i = stations_map.begin(); i != stations_map.end(); i++) {
        dijkstra_path[i->first] = new string[STATION_COUNT];
        spfa_path[i->first] = new string[STATION_COUNT];
    }

    // dijkstra
    cout << endl << "DIJKSTRA start : " << endl;
    string dijkstra_middle_station = dijkstra_find_middle_station();
    string dijkstra_destination = dijkstra_find_destination(dijkstra_middle_station);
    cout << "dijktra_middle_station : " << dijkstra_middle_station << endl;
    cout << "destination : " << dijkstra_destination << endl;

    // SPFA
    cout << endl << "SPFA start" << endl;
    string spfa_middle_station = spfa_find_middle_station();
    string spfa_destination = SPFA_find_destination(spfa_middle_station);
    cout << "spfa_middle_station : " << spfa_middle_station << endl;
    cout << "destination : " << spfa_destination << endl;

    // DFS
    cout << endl << "DFS start" << endl;
    string dfs_middle_station = dfs_find_middle_station();
    string dfs_destination = DFS_find_destination(dfs_middle_station);
    cout << "dfs_middle_station : " << dfs_middle_station << endl;
    cout << "destination : " << dfs_destination << endl;

    // 결과 출력
    dijkstra_print_result(dijkstra_destination);
    cout << "comp: " << dijkstra_cmp << endl;
    spfa_print_result(spfa_destination);
    cout << "comp: " << SPFA_cmp << endl;
    dfs_print_result(dfs_destination);
    cout << "comp: " << DFS_cmp << endl;
}

// data 입력받는 함수
void input_data() {
    ifstream readFile;
    readFile.open("input_file.txt");            // 입력 파일 이름

    string s, user_name, boarding_station;
    int index;
    while (getline(readFile, s)) {                              // 파일 입력
        index = s.find(" ");
        user_name = s.substr(0, index);                         // user name
        boarding_station = s.substr(index + 1, s.length());     // user 탑승역
        user[user_name] = boarding_station;                     // MAP user 생성
        cout << user_name << " : " << user[user_name] << endl;
    }
    readFile.close();
}

// station node 생성
void make_station_node() {
    Json::Reader reader;
    Json::Value verti;
    Json::Value stat;

    // json open
    ifstream json1;
    json1.open("vertices.json");
    ifstream json2;
    json2.open("station.json");

    // parse
    if (!reader.parse(json1, verti)) {
        cout << "Parsing Failed 'vertices.json'" << endl;
        exit(1);
    }
    if (!reader.parse(json2, stat)) {
        cout << "Parsing Failed 'station.json'" << endl;
        exit(1);
    }

    // 역 정보로 노드 생성 및 저장
    Json::Value vertice_list = verti["station"];

    for (int i = 0; i < vertice_list.size(); i++) {      // vertices.json에 저장된 모든 역 정보 가져오기

        string station_name = vertice_list[i]["station_nm"].asString();         // 역 이름
        string line_number = vertice_list[i]["line_num"].asString();            // 호선

        // 구조체 만들기 전 map에 있는 값인지 확인 : 이미 있는 역이면 호선만 추가
        if (stations_map.find(station_name) != stations_map.end()) {      // found
            int line_count = 0;
            for (int j = 0; j < 10; j++) {
                if (stations_map[station_name]->line_number[j] != "") line_count++;
            }
            stations_map[station_name]->line_number[line_count] = line_number;
            continue;
        }

        // station 구조체 생성
        station* st = new station;
        st->station_name = station_name;
        st->line_number[0] = line_number;
        st->next = nullptr;

        next_station* head = nullptr;
        next_station* tail = nullptr;

        // station에 next 추가
        auto member = stat[station_name]["time"].getMemberNames();      // 해당 역에 연결되어있는 모든 역
        for (string next_st_name : member) {
            // 새로운 next_station node 생성
            next_station* next_st = new next_station;
            next_st->station_name = next_st_name;
            next_st->time = stat[station_name]["time"][next_st_name].asInt();
            next_st->next = nullptr;

            // linked list로 연결 : 연결되어 있는 역 모두 연결
            if (head == nullptr) {
                head = next_st;
                st->next = head;
            }
            else {
                tail->next = next_st;
            }
            tail = next_st;
        }
        // map에 node 삽입
        stations_map[station_name] = st;
    }
}

// dijkstra
int dijkstra(string start, string end) {
    map<string, bool>visit;

    int j = 0;

    //다익스트라 변수 초기화
    for (auto i = stations_map.begin(); i != stations_map.end(); i++) {
        dijkstra_stations[j++] = i->first; //역 이름 초기화
        dijkstra_path[i->first][0] = start; //경로의 맨 처음은 항상 시작지점
        dijkstra_path_index[i->first] = 1; //경로의 갯수 저장할 변수. 0번은 시작지점으로 1로 초기화
    }
    for (int i = 0; i < STATION_COUNT; i++) {
        dijkstra_dist[dijkstra_stations[i]] = MAX; //MAX로 초기화
        visit[dijkstra_stations[i]] = false; //방문 기록
    }
    //시작지점 0으로 설정
    dijkstra_dist[start] = 0;

    //시작지점 저장
    next_station* start_next = stations_map[start]->next;
    //우선순위 큐(항상 distance가 가장 작은 역이 pop)
    priority_queue<station*, vector<station*>, compare> q;

    //시작지점 관련 초기화(
    while (start_next != NULL) {
        dijkstra_dist[start_next->station_name] = start_next->time;
        q.push(stations_map[start_next->station_name]);
        start_next = start_next->next;
    }

    //시작지점 방문 완료
    visit[start] = true;


    //queue가 비어있을 때까지 반복
    while (!q.empty()) {
        station* vertex = q.top();
        q.pop();
        visit[vertex->station_name] = true; //방문 표시
        next_station* through = vertex->next;   //다음 역

        while (through != NULL) {
            int time = through->time;//vertext부터 through까지 가는데 걸리는 시간(비용)
            dijkstra_cmp++;//비교회수 증가
            //방문하지 않았으며 기존의 distance보다 거쳐 가는 비용이 더 작다면 
            if (!visit[through->station_name] && dijkstra_dist[through->station_name] > dijkstra_dist[vertex->station_name] + time) { //정점에서 through까지의 시간이 temp를 거쳐 through를 가는 시간보다 짧으면 더해준다.
                //distance 갱신
                dijkstra_dist[through->station_name] = dijkstra_dist[vertex->station_name] + time;
                //경로 갱신
                dijkstra_path_index[through->station_name] = 1;
                for (int i = 1; i < dijkstra_path_index[vertex->station_name]; i++) {
                    dijkstra_path[through->station_name][dijkstra_path_index[through->station_name]++] = dijkstra_path[vertex->station_name][i];
                }
                dijkstra_path[through->station_name][dijkstra_path_index[through->station_name]++] = vertex->station_name;
                //갱신된 역 push
                q.push(stations_map[through->station_name]);
            }
            //다음 역
            through = through->next;
        }
    }
    return 0;
}
string dijkstra_find_middle_station() {
    cout << "dijkstra_find_middle_station";
    string middle_station;      // 초기지점 설정
    int far_dist = 0;           // 가장 먼 거리
    auto far_user1 = user.cbegin(), far_user2 = user.cbegin();
    for (auto ptr1 = user.cbegin(); ptr1 != user.cend(); ptr1++) {             // user1에서 모든 user와의 거리
        dijkstra(ptr1->second, "충무로");                                      // 다익스트라 구하기
        for (auto ptr2 = ptr1; ptr2 != user.cend(); ptr2++) {                   // user1 -> user2
            if (far_dist < dijkstra_dist[ptr2->second]) {                       // 최대 거리이면
                far_dist = dijkstra_dist[ptr2->second];                         // 최대 거리 업데이트     
                far_user1 = ptr1;                                      // 최대 거리 user 업데이트
                far_user2 = ptr2;
                for (int j = 0; j < dijkstra_path_index[ptr2->second]; j++) {        // user1->user2 경로
                    // 최대 경로의 중간값 찾기
                    if ((far_dist / 2) >= dijkstra_dist[dijkstra_path[ptr2->second][j]] && (far_dist / 2) < dijkstra_dist[dijkstra_path[ptr2->second][j + 1]]) {
                        middle_station = dijkstra_path[ptr2->second][j];         // 중간값일 때 station 구하기
                    }
                }
            }
        }
        cout << ".";
    }
    cout << endl;
    return middle_station;
}
string dijkstra_find_destination(string middle_station) {
    cout << "dijkstra_find_destination" << endl;
    // 초기 
    dijkstra(middle_station, "충무로");
    int sum_dist = 0;
    int min_dist = INT_MAX;

    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // 현재 중간지점에서 user까지의 가장 작은 거리 찾기
        if (min_dist > dijkstra_dist[ptr->second]) {
            min_dist = dijkstra_dist[ptr->second];
        }
    }
    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // '중간지점에서 각 user까지의 거리 - 최소거리'의 합
        sum_dist += dijkstra_dist[ptr->second] - min_dist;
    }

    // 연결된 역 확인해서 이동
    queue<string> q;
    q.push(middle_station);

    string current;
    string previous;
    string more_previous;
    while (!q.empty()) {
        current = q.front();
        q.pop();
        if (current == more_previous) break;        // 전전역과 같은지 확인, : 기준값 같을 경우 계속 반복되는 경우 방지

        next_station* ns = stations_map[current]->next;
        while (ns != NULL) {                        // 연결된 역 모두 확인
            int sum_dist_check = 0;
            min_dist = INT_MAX;
            dijkstra(ns->station_name, "충무로");      // 연결된 역들의 dijkstra
            for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // 중간지점에서 각 user간의 최소비용 구하기
                if (min_dist > dijkstra_dist[ptr->second]) {
                    min_dist = dijkstra_dist[ptr->second];
                }
            }
            for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // 'user까지의 비용 - 최소비용'의 합 구하기
                sum_dist_check += dijkstra_dist[ptr->second] - min_dist;
            }
            if (sum_dist >= sum_dist_check) {           // 현재 최소 기준점보다 작으면 push
                sum_dist = sum_dist_check;
                q.push(ns->station_name);
            }
            ns = ns->next;
        }
        more_previous = previous;
        previous = current;
    }
    return current;
}
void dijkstra_print_result(string destination) {
    cout << endl << "**dijkstra 결과 출력**" << endl;
    cout << "만날 역 : " << destination << endl;
    cout << "역까지의 경로" << endl;
    dijkstra(destination, user.cbegin()->second);
    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // 모든 유저들의 경로 출력
        cout << ptr->first << " : ";        // user 이름 출력
        cout << ptr->second << "-";         // user 탑승역 출력
        for (int i = dijkstra_path_index[ptr->second] - 1; i > 0; i--) {    // 경로 출력
            cout << dijkstra_path[ptr->second][i] << "-";
        }
        cout << dijkstra_path[ptr->second][0] << endl;
        cout << "도착지까지 걸리는 시간 : " << dijkstra_dist[ptr->second] << endl;    // 소요 시간
    }
}

// SPFA
int SPFA(string start, string end) {
    // 입력: 시작점, 끝점
    // 출력: 최단 거리
    map <string, bool> inQ; // 큐에 있는지 확인할 맵
    queue <station*> spfa_q; // 큐 생성

    // spfa_dist, spfa_path, spfa_path_index 초기화
    for (auto i = stations_map.begin(); i != stations_map.end(); i++) {
        spfa_dist[i->first] = MAX; // 거리는 모두 MAX로 초기화
        spfa_path[i->first][0] = start; // 경로 초기화
        spfa_path_index[i->first] = 1; // 경로 인덱스 값은 1로 초기화
    }

    spfa_q.push(stations_map[start]); // 시작 지점을 큐에 push

    // 시작 지점 설정
    string start_station = stations_map[start]->station_name;
    spfa_dist[start_station] = 0; // 시작 지점의 dist는 0
    inQ[start_station] = true; // 큐에 push 하였으므로 inQ true로 설정

    // 큐가 비어있을 때까지 반복
    while (!spfa_q.empty()) {
        station* current_station = spfa_q.front(); // 큐의 front에 있는 값 현재 역으로 설정
        spfa_q.pop(); // 큐에서 pop
        inQ[current_station->station_name] = false; // 큐에서 pop하였으므로 false로 설정

        next_station* adjacent_station = current_station->next; // 인접한 역 탐색하기 위해 다음 역의 포인터 저장

        // 인접한 정점이 없을 때까지 반복
        while (adjacent_station != NULL) {
            int time_cost = adjacent_station->time; // 인접한 역까지의 소요 시간(비용) 저장
            SPFA_cmp++; // distance의 비교 발생
            // 인접한 역까지의 거리가 현재 역까지의 거리에 소요 시간을 더한 값보다 크다면
            if (spfa_dist[adjacent_station->station_name] > spfa_dist[current_station->station_name] + time_cost) {
                // distance 업데이트
                spfa_dist[adjacent_station->station_name] = spfa_dist[current_station->station_name] + time_cost;
                if (!inQ[adjacent_station->station_name]) { // 탐색하려는 인접한 역이 inQ에 저장되어있지 않다면
                    spfa_q.push(stations_map[adjacent_station->station_name]); // 큐에 push

                    inQ[adjacent_station->station_name] = true; // 큐에 넣었으므로 inQ 값을 true로 설정

                    // 경로 출력
                    spfa_path_index[adjacent_station->station_name] = 1; // 인접 역의 경로 인덱스를 1로 설정

                    // 인접한 역의 경로 인덱스를 증가시키면서 현재 역의 경로 spfa_path에 저장
                    for (int i = 1; i < spfa_path_index[current_station->station_name]; i++) {
                        spfa_path[adjacent_station->station_name][spfa_path_index[adjacent_station->station_name]++]
                            = spfa_path[current_station->station_name][i];
                    }
                    spfa_path[adjacent_station->station_name][spfa_path_index[adjacent_station->station_name]++]
                        = current_station->station_name;
                }
            }
            // 다음 인접한 역 탐색
            adjacent_station = adjacent_station->next;
        }
    }
    return spfa_dist[end];
}
string spfa_find_middle_station() {
    cout << "spfa_find_middle_station";
    string middle_station;      // 중간지점
    int far_dist = 0;           // user 간 최대거리
    auto far_user1 = user.cbegin(), far_user2 = user.cbegin();

    for (auto ptr1 = user.cbegin(); ptr1 != user.cend(); ptr1++) {              // 각 user의 탑승역에서 초기지점까지 비용 구하기
        SPFA(ptr1->second, "충무로");
        for (auto ptr2 = ptr1; ptr2 != user.cend(); ptr2++) {                   // 각 유저별 거리 각각 구하기
            if (far_dist < spfa_dist[ptr2->second]) {                           // 최대 비용 시
                far_dist = spfa_dist[ptr2->second];                             // 최대 비용 업데이트
                far_user1 = ptr1;                                               // 최대비용인 user 업데이트
                far_user2 = ptr2;
                // 최대 비용인 두 user 사이의 절반의 비용을 가진 역 구하기
                for (int j = 0; j < spfa_path_index[ptr2->second]; j++) {
                    if ((far_dist / 2) >= spfa_dist[spfa_path[ptr2->second][j]] && (far_dist / 2) < spfa_dist[spfa_path[ptr2->second][j + 1]]) {   // 최대 경로의 중간값 찾기
                        middle_station = spfa_path[ptr2->second][j];         // 중간값일 때 station 구하기
                    }
                }
            }
        }
        cout << ".";
    }
    cout << endl;
    return middle_station;
}
string SPFA_find_destination(string middle_station) {
    // 초기 
    SPFA(middle_station, "충무로");
    int sum_dist = 0;
    int min_dist = INT_MAX;

    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // 초기지점과 user들간 가장 작은 비용 찾기
        if (min_dist > spfa_dist[ptr->second]) {
            min_dist = spfa_dist[ptr->second];
        }
    }
    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // '초기기점에서 user까지의 비용 - 최소비용'의 합
        sum_dist += spfa_dist[ptr->second] - min_dist;
    }

    // 연결된 역 확인해서 이동
    queue<string> q;
    q.push(middle_station);

    string current;
    string previous;
    string more_previous;
    while (!q.empty()) {
        current = q.front();
        q.pop();
        if (current == more_previous) break;        // 전전역과 같은지 확인, : 기준값 같을 경우 계속 반복되는 경우 방지        

        next_station* ns = stations_map[current]->next;
        while (ns != NULL) {                    // 연결된 모든 역 기준값 확인
            int sum_dist_check = 0;
            min_dist = INT_MAX;
            SPFA(ns->station_name, "충무로");      // 연결된 역들의 SPFA
            for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // 연결된 역에서 user까지의 최소 비용 구하기
                if (min_dist > spfa_dist[ptr->second]) {
                    min_dist = spfa_dist[ptr->second];
                }
            }
            for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // '연결된 역에서 user까지의 비용 - 최소비용'의 합
                sum_dist_check += spfa_dist[ptr->second] - min_dist;
            }
            if (sum_dist >= sum_dist_check) {                   // 현재 역의 최소 기준값보다 작으면 queue에 push
                sum_dist = sum_dist_check;
                q.push(ns->station_name);
            }
            ns = ns->next;
        }
        more_previous = previous;
        previous = current;
    }
    return current;
}
void spfa_print_result(string destination) {
    cout << endl << "**spfa 결과 출력**" << endl;
    cout << "만날 역 : " << destination << endl;       // 중간지점
    cout << "역까지의 경로" << endl;
    SPFA(destination, "충무로");
    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // 각 user 경로 출력
        cout << ptr->first << " : ";        // user 이름 출력
        cout << ptr->second << "-";         // uesr 탑승역 출력
        for (int i = spfa_path_index[ptr->second] - 1; i > 0; i--) {
            cout << spfa_path[ptr->second][i];
            if (i != 1) cout << "-";
        }
        cout << endl;
        cout << "도착지까지 걸리는 시간 : " << spfa_dist[ptr->second] << endl;     // 소요시간
    }
}

// DFS
void start_DFS(string start, string end) {
    cost = 9999;
    n = end;
    for (auto i = stations_map.begin(); i != stations_map.end(); i++) {
        dfs_map[i->first] = 0;
        dfs_dist[i->first] = MAX;
    }
    DFS(start, 0, "");
}
void DFS(string v, int sum, string s) {
    int i;
    struct next_station* dsfp;
    DFS_cmp++;
    if (dfs_dist[v] < sum)
        return;
    else {
        dfs_dist[v] = sum;
    }
    if (sum > cost)
        return;
    if (v == n) {
        if (sum < cost) {
            cost = sum;
            dfs_path = s;
        }
        return;
    }
    dsfp = stations_map[v]->next;
    while (dsfp != NULL) {
        //cout << dsfp->station_name <<endl;
        if (dfs_map[dsfp->station_name] == 1) {
            dsfp = dsfp->next;
            continue;
        }
        dfs_map[dsfp->station_name] = 1;
        DFS(dsfp->station_name, sum + dsfp->time, s + " " + dsfp->station_name);
        dfs_map[dsfp->station_name] = 0;
        dsfp = dsfp->next;
    }
}
string dfs_find_middle_station() {
    cout << "dfs_find_middle_station";
    string middle_station;
    int far_dist = 0;
    auto far_user1 = user.cbegin(), far_user2 = user.cbegin();
    for (auto ptr1 = user.cbegin(); ptr1 != user.cend(); ptr1++) {
        for (auto ptr2 = user.cbegin(); ptr2 != user.cend(); ptr2++) {
            start_DFS(ptr1->second, ptr2->second);
            if (far_dist < cost) {
                far_dist = cost;
                far_user1 = ptr1;
                far_user2 = ptr2;
                stringstream ss(dfs_path);
                string str;
                int i = 0;
                while (ss >> str) {
                    path[i] = str;
                    i++;
                }
                for (int j = 0; j < i; j++) {
                    if ((far_dist / 2) >= dfs_dist[path[j]] && (far_dist / 2) < dfs_dist[path[j + 1]])
                        middle_station = path[j];
                }
            }
        }
        cout << ".";
    }
    cout << endl;
    return middle_station;
}
string DFS_find_destination(string middle_station) {
    cout << "dfs_find_destination" << endl;
    // 초기
    int sum_dist = 0;
    int min_dist = INT_MAX;
    vector<int> v;

    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // user간의 가장 최소 비용 찾기
        start_DFS(ptr->second, middle_station);
        v.push_back(cost);
        if (min_dist > cost) {
            min_dist = cost;
        }
    }
    for (int i = 0; i < v.size(); i++) {        // '초기지점에서 user까지의 비용 - 최소비용'의 합
        sum_dist += v[i] - min_dist;
    }

    // 중간지점 업데이트
    queue<string> q;
    q.push(middle_station);

    string current;
    string previous;
    string more_previous;
    while (!q.empty()) {
        current = q.front();
        q.pop();
        if (current == more_previous) break;            // 전전역과 같은지 확인, : 기준값 같을 경우 계속 반복되는 경우 방지

        next_station* ns = stations_map[current]->next;
        while (ns != NULL) {            // 연결된 역들의 기준값 구하기
            v.clear();
            int sum_dist_check = 0;
            min_dist = INT_MAX;
            for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // 현재 중간지점에서 user까지의 최소 비용
                start_DFS(ns->station_name, ptr->second);
                v.push_back(cost);
                if (min_dist > cost) {
                    min_dist = cost;
                }
            }
            for (int i = 0; i < v.size(); i++) {            // '현재 중간지점에서 각 user까지의 비용 - 최소비용'의 합
                sum_dist_check += v[i] - min_dist;
            }
            if (sum_dist >= sum_dist_check) {               // 현재 최소 기준값보다 작으면 queue에 push
                sum_dist = sum_dist_check;
                q.push(ns->station_name);
            }
            ns = ns->next;
        }
        more_previous = previous;
        previous = current;
    }
    return current;
}
void dfs_print_result(string destination) {
    string space_delimiter = " ";
    vector<string> words{};

    cout << endl << "**dfs 결과 출력**" << endl;
    cout << "만날 역 : " << destination << endl;
    cout << "역까지의 경로" << endl;
    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // 각 user 경로 출력
        start_DFS(ptr->second, destination);

        cout << ptr->first << " : ";        // user 이름 출력
        cout << ptr->second;                // user 탑승역 출력
        string buf;
        istringstream ss(dfs_path);
        vector<string> arr;

        while (getline(ss, buf, ' ')) {
            arr.push_back(buf);
        }

        for (int i = 0; i < arr.size(); i++) {
            cout << arr[i];
            if (i != arr.size() - 1) cout << "-";
        }
        cout << endl;
        cout << "도착지까지 걸리는 시간 : " << dfs_dist[destination] << endl;     // 소요시간 출력
    }
}