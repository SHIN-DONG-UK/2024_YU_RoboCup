### `handleReturn()` 함수 문서

<br>

## 🎯 함수 목적 요약
- `ret_rot[]`, `ret_dis[]`에 저장된 **리턴 경로 정보를 따라 로봇을 복귀 주행**시키는 함수

- 각 리턴 노드 구간을 따라:
   - 필요한 만큼 **직진 이동**
   - **회전이 필요한 경우 회전 수행**

- 목적지에 도달하면 상태를 **`EXPLORE` 또는 `HDRIVE`** 로 전환

<br>

## 📥 Input: 없음
- 내부적으로 전역 변수인 `ret_rot[]`, `ret_dis[]`, `rcnt`, `rn`, `MM.det` 등을 참조함

<br>

## 핵심 로직 단계

### 1️⃣ 라인트레이싱 및 주행 상태 설정
```cpp
MM.cal_pos(800);
if (vss < 16) { vss = VSS_RETURN; }
```
- 센서 기반 위치 인식
- OLED 표시 설정

### 2️⃣ 다음 노드 감지
```cpp
if (MM.det != 2) {
    if (rn < rcnt - 2) {
        rn++;  // 다음 리턴 노드 번호
        ...
    }
}
```

### 3️⃣ 이동 전 정지 및 초기화
```cpp
memset(MM.pos, 0, sizeof(MM.pos));
memset(MM.encCnt, 0, sizeof(MM.encCnt));
while (MM.encCnt[0] + MM.encCnt[1] < 200) { delay(1); }
```
- 기존 위치/거리 정보 초기화
- 라인 인식 안정화를 위해 **살짝 전진**

### 4️⃣ 회전 명령 수행
```cpp
if (ret_rot[rn] % 2 == 1) {
    dis = 500; ang = 0;
    wd = wlim;
    MM.dis_com = 0;
    vel_dis = 1;
    while (!MM.dis_com) { delayMicroseconds(1000); }
    vel_dis = 0;
    turn(680 * (ret_rot[rn] - 2));
}
```
- 리턴 경로상 회전 정보가 `1`, `3`(좌/우)일 경우 -> 회전 수행
- `ret_rot[rn]` 값 기준으로  `turn()` 호출

### 5️⃣ 다음 이동 준비
```cpp
MM.det = 2;
```
- 주행 모드로 복귀

### 6️⃣ 마지막 리턴 노드 도착 시 상태 전환
```cpp
else {
    if (complete == 0) {
        sta = EXPLORE;
        vd = vlim;
        wd = 0;
        rn = 0;
        vss = VSS_DRIVING;
    } else {
        sta = HDRIVE;
        vd = 0;
        wd = 0;
        rn = 0;
        node_end = optimum_path(rcnt - 1);
    }
}
```
- 복귀가 완료되었을 경우,
   - 미로 탐색이 아직이면 -> 다시 상태를 `EXPLORE`로
   - 탐색 완료 상태면 -> `HDRIVE`로 전환하여 **최단 경로 주행 시작 준비**

## 📤 Output: 없음
- 전역 상태(`sta`, `vss`, `rn`, `node_end`)가 갱신되어 다음 루프에서 동작 결정

## 🧠 요약
| 기능             | 설명                                |
| -------------- | --------------------------------- |
| 🧭 현재 위치 추적    | `MM.cal_pos()`로 라인 센싱             |
| 🧶 리턴 경로 따라 이동 | `ret_rot[]`, `ret_dis[]` 기반 회전 수행 |
| ⛔ 마지막 노드 도착 확인 | `rn == rcnt - 2` 기준               |
| 🔁 상태 전환       | `EXPLORE` or `HDRIVE`             |
