// 리팩토링 목적: A* 경로 탐색 및 마이크로마우스 주행 코드를 모듈화하고 구조 분리하기
#include <micromouse.h>
#include <variable.h>
#include <RPi_Pico_TimerInterrupt.h>
#include <PriorityQueue.h>

// --- Configuration --- //
#define SIZE_X 14
#define SIZE_Y 22
#define UNIT_DIST 1250
#define COST_MOVE 10
#define COST_TURN 10

// --- 방향 배열 --- //
int8_t dirX[4] = { -1, 0, 1, 0 };
int8_t dirY[4] = { 0, -1, 0, 1 };

// --- Structs & Enums --- //
struct Node {
    uint8_t x, y, dir;
    unsigned int F, G;
};

enum Sta : uint8_t{
    EXPLORE = 0,
    RETURN = 1,
    HDRIVE = 2,
    FINISH = 99
}

enum VssState : uint8_t {
    VSS_READY = 10,        // 버튼 대기 중
    VSS_DRIVING = 11,      // 주행 중
    VSS_RETURN = 13,       // 막다른 길 → 재탐색 중
    VSS_FINISH = 15,       // 목적지 도착
    VSS_REEXPLORE = 16,    // 경로 재탐색
    VSS_COMPLETE = 17,        // 경로 없음 → 종료
    VSS_DYNAMIC = 100      // 노드 번호 기반 상태 (vss=ncnt)
};

bool compare(Node a, Node b) { return a.F < b.F; }

// --- 전역 변수 --- //
PriorityQueue<Node> open_nodes(compare);
unsigned int openF[SIZE_Y][SIZE_X];
uint8_t par_x[SIZE_Y][SIZE_X], par_y[SIZE_Y][SIZE_X];
unsigned int Total_G=65000;         // 현 상황에서 출발점-목적지 거리
bool closed[SIZE_Y][SIZE_X];            // 닫힌목록  
bool revit=0, complete=0;               // 목적지 도착후 완료 및 재탐색 유무


// 리턴경로 정보
uint8_t ret_x[256],ret_y[256];      // 리턴경로 x,y 좌표
uint8_t ret_rot[256],ret_dis[256];  // 리턴경로 회전 및 거리 정보 
uint8_t rcnt=0, rn=0;               // 리턴경로 노드갯수 및 현재 리턴노드
unsigned int ret_cost=0;            // 리턴경로의 cost (최종거리값)


// --- 로봇 --- //
uint8_t start_x = 3, start_y = 0;
uint8_t goal_x = 10, goal_y = 0;
uint8_t pos_x,  pos_y;            // 현재 위치
uint8_t pri_x,  pri_y;            // 이전 위치
uint8_t dirR = 4;
unsigned int F=0,G=0;             // 누적 F,G

// OLED Global Variable
uint8_t datad[250][11];       //OLED 표시정보 (방문노드)
uint8_t datas[250][5];        //OLED 표시정보 (최적경로)
uint8_t nns=0;

unsigned int ncnt=0;

// --- 제어 --- //
RPI_PICO_Timer ITimer0(0);

bool vel_dis=0;
int vH=120, vL=60;    //고속,저속 속도
int wL=50;
int vlim=40, wlim=70;
int vd=0, wd=0, dis=0, ang=0;
VssState vss = VSS_DYNAMIC;
Sta sta = EXPLORE;

uint8_t node=0;                 //현재 노드 번호 
uint8_t node_end=0;             //끝점 노드 번호 = optimum_path()함수의 리턴으로 넣어줌
uint8_t node_rot[256], node_dis[256];
uint8_t dv=1;


bool TimerHandler0(struct repeating_timer *t) {
    MM.Get_velo();
    if (vel_dis==0) { MM.Set_velo(vd,wd); }
    else            { MM.Set_dis(dis,ang,vd,wd,10,10); }
    return true;
}

void setup() {
    Serial.begin(9600); delay(500);
    multicore_launch_core1(main2);

    memset(openF, 255, sizeof(openF));
    open_nodes.clear();
    pos_x = start_x;
    pos_y = start_y;
    openF[start_y][start_x] = 0;

    MM.Motor_Init();
    MM.Set_velo_gain(5,10,5,10,5,150,400);
    MM.Set_dis_gain(1.,0.4,1.,0.4);
    Int_Init();
    ITimer0.attachInterruptInterval(10 * 1000, TimerHandler0);

    // --- 시작 버튼 --- //
    pinMode(1,INPUT_PULLUP);
    int  p=0;
    while(p<100)  { if (digitalRead(1)==0) {p++;}  else {p=0;}   delay(10);  }   // 버튼 1초 누를때 까지 대기
    
    vss=VSS_READY;  p=0;
    
    while(p<100)  { if (digitalRead(1)==1) {p++;}  else {p=0;}   delay(10);  }   // 버튼 때고 1초 대기
    for (p=2; p>0; p--)   { vss=p; delay(1000); }
  
    vss=VSS_DRIVING;
    
    bool p1=digitalRead(14), p2=digitalRead(15);
    vd=20;
    while (!(p1&&p2))  { p1=digitalRead(14);  p2=digitalRead(15); delay(10);  } 
    vd=vlim; MM.det=2; 
    memset(MM.encCnt,0,sizeof(MM.encCnt)); // 카운터 리셋
}

'''
1. handleExplore    -> 미로 탐색 상태
2. handleReturn     -> 탐색 후 로봇이 돌아가는 상태
3. handleDrive      -> 최단 경로 고속 주행
4. handleFinish     -> 종료
'''
void loop() {
    switch (sta) {
        case EXPLORE:   handleExplore();    break;
        case RETURN:    handleReturn();     break;
        case HDRIVE:    handleHDrive();     break;
        case FINISH:    handleFinish();     break;
    }
    delayMicroseconds(100);
}

void handleExplore() {
    ''' MM.det / bit0=1 -> 오른쪽 길 감지 / bit1=2 -> 직진 길 감지 / bit2=4 -> 왼쪽 길 감지 '''
    MM.cal_pos(800); // 라인트레이싱
    
    if (MM.det!=2) {
        '''실제 정보로 업데이트'''
        int node_dis = MM.encCnt[0]+MM.encCnt[1];               //방문노드까지 거리 카운터
        memset(MM.pos,0,sizeof(MM.pos));                        //위치제어 중단
        memset(MM.encCnt,0,sizeof(MM.encCnt));                  //엔코더 리셋
        int node_num = round( node_dis / float(UNIT_DIST));     //엔코더 거리 -> 좌표 변환
        uint8_t pri_x=pos_x, pri_y=pos_y;                       //이전위치 업데이트
        pos_x += dirX[dirR-1] * node_num;                       //현재위치
        pos_y += dirY[dirR-1] * node_num;
        closed[pos_y][pos_x] = 1;                               //실제 방문하려고 한 점만 closed
        
        uint8_t next_dir=0;                                     // 다음 이동 방향 (0:막다른길)
        
        G=G + node_num*COST_MOVE;                               // 현재 위치 G값 업데이트  
        F=G + estimate_dist(pos_x,pos_y,dest_x,dest_y, dirR);   // 현재 위치 F값 업데이트
        if (F < openF[pos_y][pos_x]) {                          // 현재 위치 F값이 이전방문보다 작다면 부모노드 및 F 업데이트
            par_x[pos_y][pos_x]=pri_x;
            par_y[pos_y][pos_x]=pri_y;
            openF[pos_y][pos_x]=F;
        }
        
        //OLED 정보 업데이트 : 방문노드 넘버, 현재위치
        datad[ncnt][0]=ncnt;  datad[ncnt][1]=pos_x;  datad[ncnt][2]=pos_y;

        // 현재위치 직전까지 방문완료 표시
//        int8_t xs=(pos_x-pri_x>0) - (pos_x-pri_x<0), ys=(pos_y-pri_y>0) - (pos_y-pri_y<0);
//        
//        if (ys==0) {  for (int p=pri_x; p !=pos_x; p=p+xs)  { closed[pos_y][p]=1; } }
//        else       {  for (int p=pri_y; p !=pos_y; p=p+ys)  { closed[p][pos_x]=1; } }
        

        // 목적지 도착 혹은 재탐색으로 이미 방문한 곳에 도착했을 때 
        if (((pos_x==dest_x)&&(pos_y==dest_y)) || (closed[pos_y][pos_x]&&revit) ) {
            closed[pos_y][pos_x]=1; revit=1;                           // 방문완료 설정후 재탐색
            int tempdir=return_path(dest_x, dest_y, start_x, start_y, 0);    
            unsigned int GG=ret_cost; 
            if (GG <  Total_G) { Total_G=GG;}                          // 출발-도착 cost 계산                                        // 출발-도착 cost 계산
            struct Node new_node=find_new_node();                      // 방문하지 않은 노드 중 최소 F 노드

            // OLED 정보 업데이트 (추출점 좌표 및 F값)
            datad[ncnt][7]=new_node.x; datad[ncnt][8]=new_node.y;   datad[ncnt][9]=highByte(new_node.F);  datad[ncnt][10]=lowByte(new_node.F);

            if (new_node.F < Total_G) {// 재탐색
                vss=VSS_REEXPLORE; G = new_node.G - COST_MOVE;   // 회전만 고려하여 도착점 G값 업데이트
                dirR=return_path(pos_x, pos_y, new_node.x, new_node.y, dirR);
                pos_x=par_x[new_node.y][new_node.x];  pos_y=par_y[new_node.y][new_node.x];
            }
            // 탐색 완료 : 출발점 복귀
            else {
                vss=VSS_COMPLETE; complete=1;   
                dirR=return_path( pos_x, pos_y, start_x, start_y, dirR );  
            }
            next_dir=ret_rot[0]; sta=RETURN;   // 리턴상태로 이동
        }
        // 일반 노드에서는 탐색
        else { 
            closed[pos_y][pos_x]=1;
            while(MM.encCnt[0]+MM.encCnt[1] < 200) { delay(1); }                // 좌우 라인이 감지되지 않도록 살짝 전진
            bitWrite(MM.det,1,(analogRead(A1)<300) || (analogRead(A2)<300));    // 직진 라인이 있는지 확인 (좌우라인은 인터럽트로 확인)
            unsigned int temp_F=65535, temp_G=0;  // 탐색에 사용할 임시 F,G값
            
            // 좌 / 직진 / 우 탐색
            // 다음 칸은 1칸 이동할 것이라고 일단 예측
            for (int p=1; p<4; p++) {
                if (bitRead(MM.det,3-p)==1) {               // 해당 방향으로 길이 존재하면
                    int8_t dirU = dirR + 2 - p;             // 해당 방향 이동시 로봇 방향
                    if (dirU>4) {dirU = dirU -4;} else if (dirU<1) {dirU=dirU+4;}
                    uint8_t next_x=pos_x+dirX[dirU-1];      // 탐색점 x좌표
                    uint8_t next_y=pos_y+dirY[dirU-1];      // 탐색점 y좌표
                    datad[ncnt][2+p]=p;                     // OLED 정보 업데이트 (회전가능 방향)
                    
                    if ( (closed[next_y][next_x]==0)||(revit) ) { // 방문하지 않은 곳 또는 목적지 도착후 재방문이면 거리 예측
                        unsigned int g = G + COST_MOVE + (dirU!=dirR)*COST_TURN;                // 임시 f,g값 (회전이 필요한 경우 코스트 추가)
                        unsigned int f = g + estimate_dist(next_x,next_y,dest_x,dest_y, dirU);   
    
                        // f값이 이전에 계산된 f보다 작은 경우에만 업데이트
                        if ( f < openF[next_y][next_x] ) {
                            openF[next_y][next_x] = f;
                            struct Node temp={ next_x, next_y, dirU, f, g};
                            open_nodes.push(temp);  
                            par_x[next_y][next_x]=pos_x;  par_y[next_y][next_x]=pos_y;
                        }
                        // 최소 예측거리에 해당하는 정보 저장
                        if (f <= temp_F )  { temp_F=f; temp_G=g; next_dir=p; }        
                    }               
                }
            }
            // 탐색결과 이동가능한 곳이 있다면, 해당 장소로 위치 업데이트
            // greedy 탐색
            if (next_dir!=0) {
                dirR = dirR + 2 - next_dir;                             // 업데이트 후 로봇 방향 
                if (dirR>4) {dirR = dirR -4;} else if (dirR<1) {dirR=dirR+4;}
                G = temp_G-COST_MOVE;                                   // 한 칸 이동량 빼기
                // OLED 정보 업데이트 : 실제 회전방향, 목표지점, 예상 F
                datad[ncnt][6]=next_dir;  datad[ncnt][7]=pos_x+dirX[dirR-1]; 
                datad[ncnt][8]=pos_y+dirY[dirR-1];   datad[ncnt][9]=highByte(temp_F);  datad[ncnt][10]=lowByte(temp_F);  
            }
            // 탐색결과 막다른길
            // open-list에서 pop
            else {
                struct Node new_node=find_new_node();                              // 방문하지 않은 노드 중 최소 F 노드 pop
                
                // 지금까지 실제 cost보다 더 큰 f는 컷 (dummy 컷)
                if ( new_node.F < Total_G ) {
                    dirR=return_path(pos_x, pos_y, new_node.x, new_node.y, dirR);   // 해당 노드까지 경로 추출  
                    G = new_node.G - COST_MOVE;                                     // 회전만 고려하여 G값 업데이트
                    pos_x=par_x[new_node.y][new_node.x];                            // 리턴위치 업데이트
                    pos_y=par_y[new_node.y][new_node.x];
                } 
                
                // 새로운 노드의 F가 현재 목적지까지 거리보다 크다면 탐색 종료
                ////////////// 여기서 dis와 rot을 구해야 함 ///////////////////////////////
                // 여기서 rcnt가 결정
                else  {  vss=VSS_RETURN; complete=1; dirR=return_path(pos_x, pos_y, start_x, start_y, dirR);}          

                next_dir = ret_rot[0]; sta = RETURN; // 리턴상태로 이동

                // OLED 정보 업데이트 :  리턴지점, 예상 F
                datad[ncnt][7]=new_node.x; datad[ncnt][8]=new_node.y;   datad[ncnt][9]=highByte(new_node.F);  datad[ncnt][10]=lowByte(new_node.F);
            }
        }

        // 결정된 정보를 바탕으로 회전 선택
        if (next_dir%2!=0) {
            dis=500; ang=0;  wd=wlim; MM.dis_com=0;  vel_dis=1;  
            while(!MM.dis_com) { delayMicroseconds(1000); } vel_dis=0; dis=0;
        }
  
        memset(MM.pos,0,sizeof(MM.pos));  memset(MM.encCnt,0,sizeof(MM.encCnt)); 
        if      (next_dir==1) { turn(-680); }
        else if (next_dir==2) { vd=vlim; wd=0; }
        else if (next_dir==3) { turn( 680); } 
        else if (next_dir==4) { turn(1300); } 
        
        ncnt++;
        dis=0; ang=0;
        MM.det=0;
        memset(MM.encCnt,0,sizeof(MM.encCnt));
    }
}

// RETURN : ret_rot에 적힌대로 돌아가는 상태
void handleReturn() {
    MM.cal_pos(800);  if(vss<16) { vss=VSS_RETURN; }
    if (MM.det!=2) { 
        if (rn<rcnt-2) {
            rn++;    // 리턴 노드 넘버
            memset(MM.pos,0,sizeof(MM.pos));                        // 위치제어 중단
            memset(MM.encCnt,0,sizeof(MM.encCnt));                  // 카운터 리셋
            while(MM.encCnt[0]+MM.encCnt[1] < 200) { delay(1); }    // 좌우라인에서 살짝 전진
        
            if (ret_rot[rn]%2==1) { //회전이 필요하다면 정의된 방향으로 회전
                dis=500; ang=0;  wd=wlim; MM.dis_com=0;  vel_dis=1;  
                while(!MM.dis_com) { delayMicroseconds(1000); } vel_dis=0;
                turn(680 *(ret_rot[rn]-2));
            }
            MM.det=2;
        }
        else {  //목적지 도착시 상태전환
            if (complete==0) {sta=EXPLORE; vd=vlim; wd=0; rn=0; vss=VSS_DRIVING; } 
            else             {sta=HDRIVE; vd=0; wd=0; rn=0; node_end = optimum_path(rcnt-1);}
        }
    }
}

// HDRIVE : 커맨드 주행
void handleHDrive() {
    vss=VSS_FINISH;
    MM.Set_velo_gain(5,10,5, 10,5, 200, 800); // 맨 뒤에 두 개가 P, D
    MM.Set_dis_gain(1.,-0.4,1.,-0.4);
    vlim=30;

    int  p=0;
    while(p<100)  { if (digitalRead(1)==0) {p++;}  else {p=0;}   delay(10);  }   // 버튼 1초 누를때 까지 대기
    vss=VSS_READY;  p=0;
    while(p<100)  { if (digitalRead(1)==1) {p++;}  else {p=0;}   delay(10);  }   // 버튼 때고 1초 대기
    for (p=2; p>0; p--)   { vss=p; delay(1000); }

    vss=VSS_DRIVING;  
    
    bool p1=digitalRead(14), p2=digitalRead(15);
    vd=20;
    while (!(p1&&p2))  { p1=digitalRead(14);  p2=digitalRead(15); delay(10);  } 
    vd=vlim; MM.det=2;
    memset(MM.encCnt,0,sizeof(MM.encCnt)); // 카운터 리셋
    
    ''' 최적 경로 주행 시작 '''
    '''    커맨드 주행     '''
    
    while(1) {
        memset(MM.encCnt,0,sizeof(MM.encCnt));     // 카운터 리셋
        //주행할 노드의 길이가 1보다 큰 경우
        if (node_dis[node]>1) { 
            // 감속 매커니즘
            if(node_dis[node] > 10) vH=180; 
            else if(node_dis[node] >2) vH=140; 
            else vH=120;
            
            dis=(node_dis[node]-1)*UNIT_DIST + 450; ang=0;   // 이동할 거리 : 노드수
            memset(MM.encCnt,0,sizeof(MM.encCnt));      // 거리 리셋
            MM.dis_com=0; vel_dis=1;                    // 거리제어

            while(MM.encCnt[0]+MM.encCnt[1]<dis) {
                MM.cal_pos(850); 
                if(vd<vH){vd = vd + dv;}
                if (MM.det!=2) {MM.det=2; MM.pos[1]=MM.pos[0]; } // 이동중 노드 무시

                delay(1); //거리 완료될동안 대기
            }   
        }

        // 마지막 노드가 아니라면 회전
        if (node<node_end) {  
            vel_dis=0; vd=vL; wd=0; dis=0; ang=0;
            MM.det=2;  
            while(MM.det==2) {MM.cal_pos(850); delay(1);}      // 노드를 만날때 까지 전진  
            memset(MM.encCnt,0,sizeof(MM.encCnt));             // 카운터 리셋
            memset(MM.pos,0,sizeof(MM.pos));                   // 라인위치 리셋
            while(MM.encCnt[0]+MM.encCnt[1] < 400) {delay(1);}   // 노드점에서 조금 더 이동
            turn( 700*( node_rot[node]-2 ) );                  // 선택방향으로 회전
            node++;                                            // 노드번호 증가
        }
                     
        // 마지막 노드라면 끝까지 전진
        else { 
            while(MM.det==2) {MM.cal_pos(850); delay(1);}               // 노드 감지때까지 유지
            vd=0; wd=0; sta=FINISH; vss=VSS_FINISH; dis=0; ang=0;       // 목적지 도착
        }
    }
}

// 회전함수
// ------------------------------------
// input :
//     ad - 회전량()
// ------------------------------------
void turn (int ad){
    memset(MM.encCnt,0,sizeof(MM.encCnt));
    ang=ad; dis=0; vel_dis=1;
    wd=wlim; vd=vlim;  MM.dis_com=0; 
    bool p1=0;
    unsigned int count=0;
    
    while(! (MM.dis_com || p1)) {
        p1=( (abs( MM.encCnt[0]-MM.encCnt[1] - ang ) < 100 ) && ((analogRead(A1) < 250) && (analogRead(A2) < 250))); 
        delayMicroseconds(1000);
        if(count>10) {if(wd>wL){wd = wd - 1;} count=0;}
        count++;
    }
    
    vel_dis=0; vd= 0; wd=0;
    memset(MM.pos,0,sizeof(MM.pos));  
    MM.cal_pos(850); 
    while(abs(MM.pos[1])>1)  { MM.cal_pos(850); delay(1);  }  
    memset(MM.pos,0,sizeof(MM.pos));  
    
    wd=0; vd=vlim; ang=0;
}

// Heuristic 계산
// --------------------------------------
// input :
//     (cx, cy) - 현재 위치
//     (gx, gy) - 목적 위치
//     dir - 로봇 현재 방향

// output :
//     manhattan distance + rotation cost
// ---------------------------------------
unsigned int estimate_dist(uint8_t cx, uint8_t cy, uint8_t gx, uint8_t gy, uint8_t dir) { 
    uint8_t xk = gx-cx, yk = gy-cy;                          // manhattan distance
    unsigned int rst = COST_TURN;
    
    if      ((xk>0)-(xk<0) + 2 == dir )     { rst = 0; }    // 초기회전이 필요하지 않는다면, 값을 0으로 초기화.
    else if ((yk>0)-(yk<0) + 3 == dir )     { rst = 0; } 
    else if ((xk==0)&&(yk==0))              { rst = 0; }    // 목적지라면 회전 0

    rst += COST_MOVE*(abs(xk) + abs(yk));
    if ( (xk != 0) && (yk != 0) ) { rst += COST_TURN; }     // 적어도 한 번 방향 전환 필요
    
    return rst;
}

// 리턴 경로 계산
// --------------------------------------
// input :
//     (cx, cy) - 현재 위치
//     (gx, gy) - 목적 위치
//     dir - 로봇 현재 방향

// output :
//     manhattan distance + rotation cost
// ---------------------------------------
uint8_t return_path(uint8_t cx, uint8_t cy, uint8_t gx, uint8_t gy, uint8_t dir) {
    uint8_t rx[200]={gx}, ry[200]={gy};    // 리턴지점 부모노드
    unsigned int p1=0, p2=0, k=1;          // 부모노드갯수, 공통부분 갯수

    memset(ret_x,0,sizeof(ret_x));memset(ret_y,0,sizeof(ret_y)); 
    memset(ret_rot,0,sizeof(ret_rot)); memset(ret_dis,0,sizeof(ret_dis));
    // 현재위치 부모노드
    ret_x[0]=cx;  ret_y[0]=cy;
    while ( (cx!=start_x)||(cy!=start_y) ) {  
        p1++;
        uint8_t temp_x=par_x[cy][cx], temp_y=par_y[cy][cx]; 
        ret_x[p1]=temp_x; ret_y[p1]=temp_y;
        cx=temp_x; cy=temp_y;    
    }

    // 리턴지점이 출발점이 아닌 경우  
    if ((gx!=start_x)||(gy!=start_y)) {   
        // 리턴지점 부모노드
        while ( (gx!=start_x)||(gy!=start_y) ) {
            p2++;
            uint8_t temp_x=par_x[gy][gx], temp_y=par_y[gy][gx];
            rx[p2]=temp_x; ry[p2]=temp_y;
            gx=temp_x; gy=temp_y;
        }
        while(!((ret_x[p1-k]!=rx[p2-k])||(ret_y[p1-k]!=ry[p2-k])))  { k++; }                        // 공통부분 감지
        for (int i=0; i<=p2-k; i++)  { ret_x[p1-k+2+i]=rx[p2-k-i]; ret_y[p1-k+2+i]=ry[p2-k-i]; }    // 공통부분 제외 후 합치기                               
        rcnt=p1+p2-2*k+3;                                                                           // 방문할 노드의 갯수
    }      
    else rcnt=p1+1;

    ret_cost=0;           // 코스트값 리셋
    // 리턴 회전정보 및 코스트값 계산
    for (int p=0; p<rcnt-1; p++) {  
        int8_t xk=ret_x[p+1]-ret_x[p],  yk=ret_y[p+1]-ret_y[p];  // 다음점과의 거리차이
        
        uint8_t dir2= 2*(yk>0) +  (xk>0)-(xk<0) + 2;             // 다음점 이동후 로봇방향
        if (dir==0) {dir=dir2;}                                  // 방향 0 입력시, 초기회전 고려하지 않음
        int8_t r21=dir-dir2+2;                                   // 다음점으로 가기위한 회전유무 (1좌2직3우4U)
        if (r21<1) {r21=r21+4;} 
        else if (r21>4) {r21=r21-4;}

        ret_cost = ret_cost + COST_MOVE*( abs(xk)+abs(yk) ) + (dir!=dir2)*COST_TURN;
        ret_rot[p]=r21;
        ret_dis[p] = abs(xk) + abs(yk);
        dir=dir2;
    }
    
    return dir;   // 리턴 완료 후 로봇방향
}

// 우선순위 큐에서 새로운 노드 얻기
// --------------------------------------
// output : 새로운 노드
// --------------------------------------
struct Node find_new_node() {
    bool done=0;
    struct Node bn;

    while(!done) {
        bn=open_nodes.pop();       // 최소 F값 노드 Pop
        if (closed[bn.y][bn.x]==0) { done=1; }
    }
    
    return bn;
}

// 최적경로
// cnt는 부모노드 개수가 아니라 rot, dis 개수
uint8_t optimum_path(uint8_t cnt) { 
    memset(node_rot,0,sizeof(node_rot)); 
    memset(node_dis,0,sizeof(node_dis));
    // cnt는 rot, dis의 길이
    // 이때 직진이 포함되어 있는 rot, dis의 길이임
    // optimum_path가 직진이 포함된 rot, dis의 직을 없앰
    uint8_t i=0;
    while(cnt>0) {
        while(ret_rot[cnt-1]==2 && cnt -2>=0){ret_dis[cnt-2] += ret_dis[cnt-1]; cnt--;}
        node_dis[i]=ret_dis[cnt-1];
        if(ret_rot[cnt-1]==1){node_rot[i]=3;} // 방향 반대
        else if(ret_rot[cnt-1]==3){node_rot[i]=1;}
        cnt--; i++;
    }
    return i;
}

// OLED 표시
void main2() {
    bool dps=0;  // 0:방문노드 출력 1:최적경로 출력
    bool chg=1;
    int8_t cnt=0, c1=0, dcnt;
    String q1,q2;
   
    MM.OLED_Init();
    while(1) { 
        if (vss>10) {
            if (digitalRead(1)==0) {cnt++; if ((cnt>10)&&chg) {c1++; cnt=0; chg=0; if (c1>dcnt-1) {c1=0;} } }
            else {cnt=0; chg=1;}  
        }

        if (dps==0) { 
            dcnt=ncnt;
            String dik;
            for (int p=3; p<=5; p++) {
                if      ( datad[c1][6]==p-2) {dik=dik+"O"; }
                else if ( datad[c1][p] >  0) {dik=dik+"o"; }
                else                         {dik=dik+"_"; }    
            }
        
            q1=String(datad[c1][0])+" "+String(datad[c1][1])+","+String(datad[c1][2])+" "+dik;
            q2=String(dcnt)+" "+String(datad[c1][7])+","+String(datad[c1][8])+" "+String(word(datad[c1][9],datad[c1][10]));
        }
        else { 
            dcnt=rcnt;
            q1=String(c1)+ " (" + String(datas[c1][0]) + "," + String(datas[c1][1]) + ")";
            //q2=String(dcnt)+ "  R" + String(datas[c1][2]) + "," + " D" + String(datas[c1][3]);
            q2=String(dcnt)+ " (" + String(datas[c1+1][0]) + "," + String(datas[c1+1][1]) + ")";
        }
        
        MM.OLED_D0(vss);  MM.OLED_D1(q1);   MM.OLED_D2(q2);  MM.OLED_Disp();  
        delay(20); 
    } 
}