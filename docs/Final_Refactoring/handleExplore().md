### 🚗 `handleExplore()` 함수 문서

<br>

## 🎯 함수 목적 요약
- 마이크로마우스가 **미로를 탐색**하며,

   - 현재 위치를 기반으로 A* 노드 확장

   - 주변 노드를 평가하여 다음 이동 방향 결정

   - 목적지 도달 시 리턴 경로 계산

- 장애물, 갈림길, 막다른 길 등을 판단하여

   - 적절한 방향을 선택하고,

   - 다음 상태(`sta`)로 전환하는 **탐색 상태 제어 함수**

---

<br>

## 📥 Input: 없음
- 내부적으로 `MM.det`, `MM.encCnt`, `par_x`, `par_y`, `openF` 등 전역 상태 참조

## 핵심 로직

### 1️⃣ 라인트레이싱 및 노드 도착 감지
```cpp
MM.cal_pos(800);
if (MM.det != 2) {
...
}
```
- 센서 값(`MM.det`)을 기준으로, 갈림길에 도착했는지 판단
- `MM.det`: bit flag (bit0: 오른쪽, bit1: 직진, bit2: 왼쪽)

### 2️⃣ 전처리
> 마이크로마우스는 다음 노드로 이동할 때 **한 칸인 줄 알고 이동**하기 때문에, 실제 이동량과 코스트로 업데이트하는 로직이 필요

```cpp
node_dis = MM.encCnt[0] + MM.encCnt[1];
node_num = round(node_dis / float(unit_node));
```
- 엔코더 값 기반 거리 계산 -> 실제 몇 칸 이동했는지 정수로 변환

```cpp
pos_x += dirX[dirR-1] * node_num;
pos_y += dirY[dirR-1] * node_num;
```
- 현재 방향(`dirR`)을 기준으로 위치 갱신

```cpp
G = G + node_num * COST_MOVE;
F = G + estimate_dist(...);

if (F < openF[pos_y][pos_x]) {
    par_x[pos_y][pos_x] = pri_x;
    par_y[pos_y][pos_x] = pri_y;
    openF[pos_y][pos_x] = F;
}
```
- G : 현재까지 실제 이동 비용
- F : G + 휴리스틱
- 더 나은 경로라면 부모 정보와 F값 갱신

### 3️⃣ 목적지 도착 or 재방문 처리
```cpp
if ((pos_x==dest_x && pos_y==dest_y) || (closed[pos_y][pos_x] && revit)) {
    ...
    if (new_node.F < Total_G) { ... }
    else { ... }
    sta = RETURN;
}
```
- 도착했거나 재방문한 경우:
   - `return_path()`로 리턴 경로 생성
   - 다음 탐색 노드(`find_new_node()`) 선택
   - **더 나은 후보 노드가 있으면 -> 재탐색
   - **없으면 탐색 완료** -> 출발점으로 복귀

### 4️⃣ **일반 탐색 : 주변 노드 탐색 (좌/직진/우)**
```cpp
for (int p=1; p<4; p++) {
    if (bitRead(MM.det, 3-p) == 1) {
        ...
        if (closed[next_y][next_x] == 0 || revit) {
            ...
            if (f < openF[next_y][next_x]) {
                open_nodes.push(...);
            }
        }
    }
}
```
- 가능한 방향을 탐색해 `open_nodes`에 푸시
- 현재 3방향 중 가장 코스트가 작은 위치로 그리디하게 결정

### 5️⃣ 이동 방향 결정
```cpp
if (next_dir != 0) {
    // 방향 갱신, 위치 업데이트, G값 보정
}
else {
    // 막다른 길: 새 노드 pop 및 복귀 처리
}
```
- 가능한 경로가 있다면 -> 이동
   - 이동 후 로봇 방향 갱신
   - 한 칸보다 더 이동할 수 있기 때문에, G값 보정

- 없다면 -> open-list에서 노드 꺼내서 이동

### 6️⃣ 결정된 방향으로 물리적 이동 명령
```cpp
if (next_dir % 2 != 0) { // 회전 방향
    turn(...)
}
else { // 직진
    vd = vlim; wd = 0;
}
```
- `turn()` 호출

### 7️⃣ 탐색 정보 업데이트
```cpp
ncnt++;
MM.det = 0;
memset(MM.encCnt, 0, sizeof(MM.encCnt));
```
- 노드 번호 증가, 센서 플래그 초기화, 다음 주기 준비

---

<br>

## 📤 Output: 없음
대신 전역 상태(sta, vss, par_x/y, openF, ret_rot/dis)가 갱신되어
다음 상태 전환(RETURN, HDRIVE 등)에 영향을 줌

<br>

## 참고
### 회전 코드 표 (탐색용)

| `next_dir` | 의미      | `turn()` 인자 |
| ---------- | ------- | ----------- |
| 1          | 좌 90°   | -680        |
| 2          | 직진      | —           |
| 3          | 우 90°   | +680        |
| 4          | U턴 180° | 1300        |

---

### 상태 전환 요약

| 조건 | 다음 상태 | 설명 |
| - | - | - |
| `next_dir` 결정 후 `RETURN` 플래그 | **RETURN** | 백트래킹 경로 실행 |
| 완주 & 출발점 복귀 완료 | **HDRIVE** | 최단 경로 주행 준비 |

## 🧠 요약
| 기능          | 설명                           |
| ----------- | ---------------------------- |
| 🔍 노드 도착 판단 | `MM.det != 2` (갈림길 도착)       |
| 📏 위치 계산    | 엔코더 기반 거리 계산 후 좌표 갱신         |
| 📚 F값 계산    | G + 휴리스틱 계산, 우선순위 갱신         |
| 🧭 방향 선택    | 좌/직/우 방향 후보 평가 및 `greedy` 선택 |
| ⛔ 막다른 길 처리  | `open_nodes.pop()` 또는 복귀     |
| 🔄 상태 전환    | `sta = RETURN` or 계속 탐색      |
