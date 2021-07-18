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

    // ��� station�� node�� ����
    make_station_node();

    // dijkstra & spfa �����Ҵ�
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

    // ��� ���
    dijkstra_print_result(dijkstra_destination);
    cout << "comp: " << dijkstra_cmp << endl;
    spfa_print_result(spfa_destination);
    cout << "comp: " << SPFA_cmp << endl;
    dfs_print_result(dfs_destination);
    cout << "comp: " << DFS_cmp << endl;
}

// data �Է¹޴� �Լ�
void input_data() {
    ifstream readFile;
    readFile.open("input_file.txt");            // �Է� ���� �̸�

    string s, user_name, boarding_station;
    int index;
    while (getline(readFile, s)) {                              // ���� �Է�
        index = s.find(" ");
        user_name = s.substr(0, index);                         // user name
        boarding_station = s.substr(index + 1, s.length());     // user ž�¿�
        user[user_name] = boarding_station;                     // MAP user ����
        cout << user_name << " : " << user[user_name] << endl;
    }
    readFile.close();
}

// station node ����
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

    // �� ������ ��� ���� �� ����
    Json::Value vertice_list = verti["station"];

    for (int i = 0; i < vertice_list.size(); i++) {      // vertices.json�� ����� ��� �� ���� ��������

        string station_name = vertice_list[i]["station_nm"].asString();         // �� �̸�
        string line_number = vertice_list[i]["line_num"].asString();            // ȣ��

        // ����ü ����� �� map�� �ִ� ������ Ȯ�� : �̹� �ִ� ���̸� ȣ���� �߰�
        if (stations_map.find(station_name) != stations_map.end()) {      // found
            int line_count = 0;
            for (int j = 0; j < 10; j++) {
                if (stations_map[station_name]->line_number[j] != "") line_count++;
            }
            stations_map[station_name]->line_number[line_count] = line_number;
            continue;
        }

        // station ����ü ����
        station* st = new station;
        st->station_name = station_name;
        st->line_number[0] = line_number;
        st->next = nullptr;

        next_station* head = nullptr;
        next_station* tail = nullptr;

        // station�� next �߰�
        auto member = stat[station_name]["time"].getMemberNames();      // �ش� ���� ����Ǿ��ִ� ��� ��
        for (string next_st_name : member) {
            // ���ο� next_station node ����
            next_station* next_st = new next_station;
            next_st->station_name = next_st_name;
            next_st->time = stat[station_name]["time"][next_st_name].asInt();
            next_st->next = nullptr;

            // linked list�� ���� : ����Ǿ� �ִ� �� ��� ����
            if (head == nullptr) {
                head = next_st;
                st->next = head;
            }
            else {
                tail->next = next_st;
            }
            tail = next_st;
        }
        // map�� node ����
        stations_map[station_name] = st;
    }
}

// dijkstra
int dijkstra(string start, string end) {
    map<string, bool>visit;

    int j = 0;

    //���ͽ�Ʈ�� ���� �ʱ�ȭ
    for (auto i = stations_map.begin(); i != stations_map.end(); i++) {
        dijkstra_stations[j++] = i->first; //�� �̸� �ʱ�ȭ
        dijkstra_path[i->first][0] = start; //����� �� ó���� �׻� ��������
        dijkstra_path_index[i->first] = 1; //����� ���� ������ ����. 0���� ������������ 1�� �ʱ�ȭ
    }
    for (int i = 0; i < STATION_COUNT; i++) {
        dijkstra_dist[dijkstra_stations[i]] = MAX; //MAX�� �ʱ�ȭ
        visit[dijkstra_stations[i]] = false; //�湮 ���
    }
    //�������� 0���� ����
    dijkstra_dist[start] = 0;

    //�������� ����
    next_station* start_next = stations_map[start]->next;
    //�켱���� ť(�׻� distance�� ���� ���� ���� pop)
    priority_queue<station*, vector<station*>, compare> q;

    //�������� ���� �ʱ�ȭ(
    while (start_next != NULL) {
        dijkstra_dist[start_next->station_name] = start_next->time;
        q.push(stations_map[start_next->station_name]);
        start_next = start_next->next;
    }

    //�������� �湮 �Ϸ�
    visit[start] = true;


    //queue�� ������� ������ �ݺ�
    while (!q.empty()) {
        station* vertex = q.top();
        q.pop();
        visit[vertex->station_name] = true; //�湮 ǥ��
        next_station* through = vertex->next;   //���� ��

        while (through != NULL) {
            int time = through->time;//vertext���� through���� ���µ� �ɸ��� �ð�(���)
            dijkstra_cmp++;//��ȸ�� ����
            //�湮���� �ʾ����� ������ distance���� ���� ���� ����� �� �۴ٸ� 
            if (!visit[through->station_name] && dijkstra_dist[through->station_name] > dijkstra_dist[vertex->station_name] + time) { //�������� through������ �ð��� temp�� ���� through�� ���� �ð����� ª���� �����ش�.
                //distance ����
                dijkstra_dist[through->station_name] = dijkstra_dist[vertex->station_name] + time;
                //��� ����
                dijkstra_path_index[through->station_name] = 1;
                for (int i = 1; i < dijkstra_path_index[vertex->station_name]; i++) {
                    dijkstra_path[through->station_name][dijkstra_path_index[through->station_name]++] = dijkstra_path[vertex->station_name][i];
                }
                dijkstra_path[through->station_name][dijkstra_path_index[through->station_name]++] = vertex->station_name;
                //���ŵ� �� push
                q.push(stations_map[through->station_name]);
            }
            //���� ��
            through = through->next;
        }
    }
    return 0;
}
string dijkstra_find_middle_station() {
    cout << "dijkstra_find_middle_station";
    string middle_station;      // �ʱ����� ����
    int far_dist = 0;           // ���� �� �Ÿ�
    auto far_user1 = user.cbegin(), far_user2 = user.cbegin();
    for (auto ptr1 = user.cbegin(); ptr1 != user.cend(); ptr1++) {             // user1���� ��� user���� �Ÿ�
        dijkstra(ptr1->second, "�湫��");                                      // ���ͽ�Ʈ�� ���ϱ�
        for (auto ptr2 = ptr1; ptr2 != user.cend(); ptr2++) {                   // user1 -> user2
            if (far_dist < dijkstra_dist[ptr2->second]) {                       // �ִ� �Ÿ��̸�
                far_dist = dijkstra_dist[ptr2->second];                         // �ִ� �Ÿ� ������Ʈ     
                far_user1 = ptr1;                                      // �ִ� �Ÿ� user ������Ʈ
                far_user2 = ptr2;
                for (int j = 0; j < dijkstra_path_index[ptr2->second]; j++) {        // user1->user2 ���
                    // �ִ� ����� �߰��� ã��
                    if ((far_dist / 2) >= dijkstra_dist[dijkstra_path[ptr2->second][j]] && (far_dist / 2) < dijkstra_dist[dijkstra_path[ptr2->second][j + 1]]) {
                        middle_station = dijkstra_path[ptr2->second][j];         // �߰����� �� station ���ϱ�
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
    // �ʱ� 
    dijkstra(middle_station, "�湫��");
    int sum_dist = 0;
    int min_dist = INT_MAX;

    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // ���� �߰��������� user������ ���� ���� �Ÿ� ã��
        if (min_dist > dijkstra_dist[ptr->second]) {
            min_dist = dijkstra_dist[ptr->second];
        }
    }
    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // '�߰��������� �� user������ �Ÿ� - �ּҰŸ�'�� ��
        sum_dist += dijkstra_dist[ptr->second] - min_dist;
    }

    // ����� �� Ȯ���ؼ� �̵�
    queue<string> q;
    q.push(middle_station);

    string current;
    string previous;
    string more_previous;
    while (!q.empty()) {
        current = q.front();
        q.pop();
        if (current == more_previous) break;        // �������� ������ Ȯ��, : ���ذ� ���� ��� ��� �ݺ��Ǵ� ��� ����

        next_station* ns = stations_map[current]->next;
        while (ns != NULL) {                        // ����� �� ��� Ȯ��
            int sum_dist_check = 0;
            min_dist = INT_MAX;
            dijkstra(ns->station_name, "�湫��");      // ����� ������ dijkstra
            for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // �߰��������� �� user���� �ּҺ�� ���ϱ�
                if (min_dist > dijkstra_dist[ptr->second]) {
                    min_dist = dijkstra_dist[ptr->second];
                }
            }
            for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // 'user������ ��� - �ּҺ��'�� �� ���ϱ�
                sum_dist_check += dijkstra_dist[ptr->second] - min_dist;
            }
            if (sum_dist >= sum_dist_check) {           // ���� �ּ� ���������� ������ push
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
    cout << endl << "**dijkstra ��� ���**" << endl;
    cout << "���� �� : " << destination << endl;
    cout << "�������� ���" << endl;
    dijkstra(destination, user.cbegin()->second);
    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // ��� �������� ��� ���
        cout << ptr->first << " : ";        // user �̸� ���
        cout << ptr->second << "-";         // user ž�¿� ���
        for (int i = dijkstra_path_index[ptr->second] - 1; i > 0; i--) {    // ��� ���
            cout << dijkstra_path[ptr->second][i] << "-";
        }
        cout << dijkstra_path[ptr->second][0] << endl;
        cout << "���������� �ɸ��� �ð� : " << dijkstra_dist[ptr->second] << endl;    // �ҿ� �ð�
    }
}

// SPFA
int SPFA(string start, string end) {
    // �Է�: ������, ����
    // ���: �ִ� �Ÿ�
    map <string, bool> inQ; // ť�� �ִ��� Ȯ���� ��
    queue <station*> spfa_q; // ť ����

    // spfa_dist, spfa_path, spfa_path_index �ʱ�ȭ
    for (auto i = stations_map.begin(); i != stations_map.end(); i++) {
        spfa_dist[i->first] = MAX; // �Ÿ��� ��� MAX�� �ʱ�ȭ
        spfa_path[i->first][0] = start; // ��� �ʱ�ȭ
        spfa_path_index[i->first] = 1; // ��� �ε��� ���� 1�� �ʱ�ȭ
    }

    spfa_q.push(stations_map[start]); // ���� ������ ť�� push

    // ���� ���� ����
    string start_station = stations_map[start]->station_name;
    spfa_dist[start_station] = 0; // ���� ������ dist�� 0
    inQ[start_station] = true; // ť�� push �Ͽ����Ƿ� inQ true�� ����

    // ť�� ������� ������ �ݺ�
    while (!spfa_q.empty()) {
        station* current_station = spfa_q.front(); // ť�� front�� �ִ� �� ���� ������ ����
        spfa_q.pop(); // ť���� pop
        inQ[current_station->station_name] = false; // ť���� pop�Ͽ����Ƿ� false�� ����

        next_station* adjacent_station = current_station->next; // ������ �� Ž���ϱ� ���� ���� ���� ������ ����

        // ������ ������ ���� ������ �ݺ�
        while (adjacent_station != NULL) {
            int time_cost = adjacent_station->time; // ������ �������� �ҿ� �ð�(���) ����
            SPFA_cmp++; // distance�� �� �߻�
            // ������ �������� �Ÿ��� ���� �������� �Ÿ��� �ҿ� �ð��� ���� ������ ũ�ٸ�
            if (spfa_dist[adjacent_station->station_name] > spfa_dist[current_station->station_name] + time_cost) {
                // distance ������Ʈ
                spfa_dist[adjacent_station->station_name] = spfa_dist[current_station->station_name] + time_cost;
                if (!inQ[adjacent_station->station_name]) { // Ž���Ϸ��� ������ ���� inQ�� ����Ǿ����� �ʴٸ�
                    spfa_q.push(stations_map[adjacent_station->station_name]); // ť�� push

                    inQ[adjacent_station->station_name] = true; // ť�� �־����Ƿ� inQ ���� true�� ����

                    // ��� ���
                    spfa_path_index[adjacent_station->station_name] = 1; // ���� ���� ��� �ε����� 1�� ����

                    // ������ ���� ��� �ε����� ������Ű�鼭 ���� ���� ��� spfa_path�� ����
                    for (int i = 1; i < spfa_path_index[current_station->station_name]; i++) {
                        spfa_path[adjacent_station->station_name][spfa_path_index[adjacent_station->station_name]++]
                            = spfa_path[current_station->station_name][i];
                    }
                    spfa_path[adjacent_station->station_name][spfa_path_index[adjacent_station->station_name]++]
                        = current_station->station_name;
                }
            }
            // ���� ������ �� Ž��
            adjacent_station = adjacent_station->next;
        }
    }
    return spfa_dist[end];
}
string spfa_find_middle_station() {
    cout << "spfa_find_middle_station";
    string middle_station;      // �߰�����
    int far_dist = 0;           // user �� �ִ�Ÿ�
    auto far_user1 = user.cbegin(), far_user2 = user.cbegin();

    for (auto ptr1 = user.cbegin(); ptr1 != user.cend(); ptr1++) {              // �� user�� ž�¿����� �ʱ��������� ��� ���ϱ�
        SPFA(ptr1->second, "�湫��");
        for (auto ptr2 = ptr1; ptr2 != user.cend(); ptr2++) {                   // �� ������ �Ÿ� ���� ���ϱ�
            if (far_dist < spfa_dist[ptr2->second]) {                           // �ִ� ��� ��
                far_dist = spfa_dist[ptr2->second];                             // �ִ� ��� ������Ʈ
                far_user1 = ptr1;                                               // �ִ����� user ������Ʈ
                far_user2 = ptr2;
                // �ִ� ����� �� user ������ ������ ����� ���� �� ���ϱ�
                for (int j = 0; j < spfa_path_index[ptr2->second]; j++) {
                    if ((far_dist / 2) >= spfa_dist[spfa_path[ptr2->second][j]] && (far_dist / 2) < spfa_dist[spfa_path[ptr2->second][j + 1]]) {   // �ִ� ����� �߰��� ã��
                        middle_station = spfa_path[ptr2->second][j];         // �߰����� �� station ���ϱ�
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
    // �ʱ� 
    SPFA(middle_station, "�湫��");
    int sum_dist = 0;
    int min_dist = INT_MAX;

    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // �ʱ������� user�鰣 ���� ���� ��� ã��
        if (min_dist > spfa_dist[ptr->second]) {
            min_dist = spfa_dist[ptr->second];
        }
    }
    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // '�ʱ�������� user������ ��� - �ּҺ��'�� ��
        sum_dist += spfa_dist[ptr->second] - min_dist;
    }

    // ����� �� Ȯ���ؼ� �̵�
    queue<string> q;
    q.push(middle_station);

    string current;
    string previous;
    string more_previous;
    while (!q.empty()) {
        current = q.front();
        q.pop();
        if (current == more_previous) break;        // �������� ������ Ȯ��, : ���ذ� ���� ��� ��� �ݺ��Ǵ� ��� ����        

        next_station* ns = stations_map[current]->next;
        while (ns != NULL) {                    // ����� ��� �� ���ذ� Ȯ��
            int sum_dist_check = 0;
            min_dist = INT_MAX;
            SPFA(ns->station_name, "�湫��");      // ����� ������ SPFA
            for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // ����� ������ user������ �ּ� ��� ���ϱ�
                if (min_dist > spfa_dist[ptr->second]) {
                    min_dist = spfa_dist[ptr->second];
                }
            }
            for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // '����� ������ user������ ��� - �ּҺ��'�� ��
                sum_dist_check += spfa_dist[ptr->second] - min_dist;
            }
            if (sum_dist >= sum_dist_check) {                   // ���� ���� �ּ� ���ذ����� ������ queue�� push
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
    cout << endl << "**spfa ��� ���**" << endl;
    cout << "���� �� : " << destination << endl;       // �߰�����
    cout << "�������� ���" << endl;
    SPFA(destination, "�湫��");
    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // �� user ��� ���
        cout << ptr->first << " : ";        // user �̸� ���
        cout << ptr->second << "-";         // uesr ž�¿� ���
        for (int i = spfa_path_index[ptr->second] - 1; i > 0; i--) {
            cout << spfa_path[ptr->second][i];
            if (i != 1) cout << "-";
        }
        cout << endl;
        cout << "���������� �ɸ��� �ð� : " << spfa_dist[ptr->second] << endl;     // �ҿ�ð�
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
    // �ʱ�
    int sum_dist = 0;
    int min_dist = INT_MAX;
    vector<int> v;

    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // user���� ���� �ּ� ��� ã��
        start_DFS(ptr->second, middle_station);
        v.push_back(cost);
        if (min_dist > cost) {
            min_dist = cost;
        }
    }
    for (int i = 0; i < v.size(); i++) {        // '�ʱ��������� user������ ��� - �ּҺ��'�� ��
        sum_dist += v[i] - min_dist;
    }

    // �߰����� ������Ʈ
    queue<string> q;
    q.push(middle_station);

    string current;
    string previous;
    string more_previous;
    while (!q.empty()) {
        current = q.front();
        q.pop();
        if (current == more_previous) break;            // �������� ������ Ȯ��, : ���ذ� ���� ��� ��� �ݺ��Ǵ� ��� ����

        next_station* ns = stations_map[current]->next;
        while (ns != NULL) {            // ����� ������ ���ذ� ���ϱ�
            v.clear();
            int sum_dist_check = 0;
            min_dist = INT_MAX;
            for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // ���� �߰��������� user������ �ּ� ���
                start_DFS(ns->station_name, ptr->second);
                v.push_back(cost);
                if (min_dist > cost) {
                    min_dist = cost;
                }
            }
            for (int i = 0; i < v.size(); i++) {            // '���� �߰��������� �� user������ ��� - �ּҺ��'�� ��
                sum_dist_check += v[i] - min_dist;
            }
            if (sum_dist >= sum_dist_check) {               // ���� �ּ� ���ذ����� ������ queue�� push
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

    cout << endl << "**dfs ��� ���**" << endl;
    cout << "���� �� : " << destination << endl;
    cout << "�������� ���" << endl;
    for (auto ptr = user.cbegin(); ptr != user.cend(); ptr++) {     // �� user ��� ���
        start_DFS(ptr->second, destination);

        cout << ptr->first << " : ";        // user �̸� ���
        cout << ptr->second;                // user ž�¿� ���
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
        cout << "���������� �ɸ��� �ð� : " << dfs_dist[destination] << endl;     // �ҿ�ð� ���
    }
}