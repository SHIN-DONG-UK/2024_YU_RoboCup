### 🔁 `return_path()` 함수 문서

## 🎯 함수 목적 요약
- 현재 위치 `(cx, cy)`에서 목표 위치 `(gx, gy)`까지 이동하기 위한 경로를 `ret_x`, `ret_y`, `ret_rot`, `ret_dis`에 기록하고,
이동 방향 및 회전 정보를 계산하여 반환하는 함수

- 현재 위치에서 시작점으로의 **경로 A**와 목표 위치에서 시작점으로의 **경로 B**를 구하고 둘을 합치기 위함

### 🛤️ 두 경로 정의
| 경로 종류 | 의미| 저장 위치|
| - | - | - |
| 🅰️ A 경로  | 현재 위치 `(cx, cy)` → 출발점 `(stat_x, stat_y)` | `ret_x[]`, `ret_y[]` |
| 🅱️ B 경로  | 목표 위치 `(gx, gy)` → 출발점 `(stat_x, stat_y)` | `rx[]`, `ry[]`       |

<br>

## 📥 **input** : `(cx, cy)`, `(gx, gy)`, `dir`
```cpp
uint8_t return_path(uint8_t cx, uint8_t cy, uint8_t gx, uint8_t gy, uint8_t dir)
```
- 📍 `(cx, cy)` : 함수 호출 당시 로봇의 위치

- 🎯 `(gx, gy)` : 목표 위치

- 🧭 `dir` : 함수 호출 당시 로봇의 방향

<br>

## ⚙️ 핵심 로직
### 1️⃣ 현재 위치 -> 시작점까지의 경로 복원
```cpp
ret_x[0] = cx; ret_y[0] = cy;
while ((cx != stat_x) || (cy != stat_y)) {
    ...
    ret_x[p1] = temp_x; ret_y[p1] = temp_y;
}
```
- 🔙 `par_x`, `par_y`를 따라 거슬러 올라가면서 부모 노드 경로(`ret_x`, `ret_y`)를 구성

### 2️⃣ 목적지 -> 시작점까지의 경로 복원
```cpp
while ((gx != stat_x) || (gy != stat_y)) {
    ...
    rx[p2] = temp_x; ry[p2] = temp_y;
}
```

- `rx[]`, `ry[]`에 저장

### 3️⃣ ✂️ 두 경로의 공통 부분 제거 후 경로 연결
```cpp
while (!(ret_x[p1-k] != rx[p2-k] || ret_y[p1-k] != ry[p2-k])) { k++; }

for (int i = 0; i <= p2 - k; i++) {
    ret_x[p1 - k + 2 + i] = rx[p2 - k - i];
    ret_y[p1 - k + 2 + i] = ry[p2 - k - i];
}
```
- **두 경로가 겹치는 지점을 찾아** 중복 없이 하나의 경로로 병합

- 🧱 스택 구조 활용
    - `ret_[]`과 `r_[]`배열을 스택이라고 하면, `top`에는 시작점이 있음
    - `top`에서부터 내려가면서 달라질 때까지 반복

- 공통 개수를 찾았으면 반복문으로 ret_배열에 덮어 씌우기

### 4️⃣ 🔄 방향 정보 및 이동 거리 계산, 저장
```cpp
for (int p = 0; p < rcnt - 1; p++) {
    ...
    ret_rot[p] = r21;
    ret_dis[p] = 이동 거리;
}
```

- 🔃 `ret_rot[]`: 다음 노드로 이동하기 위해 필요한 회전 (1: 좌, 2: 직, 3: 우, 4: U턴)
- 📏 `ret_dis[]`: 직진 거리 (기본 단위 = 셀 개수)

## 📤 **output** : `dir`
- 리턴 완료 후 로봇 방향을 로봇 상태 변수에 저장해놓기 위함
