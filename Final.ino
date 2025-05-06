#include <micromouse.h>
#include <variable.h>
#include <RPi_Pico_TimerInterrupt.h>
#include <PriorityQueue.h>

#define size_x 14   // x사이즈
#define size_y 22   // y사이즈
//#define size_x 7   // x사이즈
//#define size_y 7   // y사이즈


// A* Global Variable
struct node {uint8_t x,y,dir;   unsigned int F,G;};   //노드정보 구조체 (위치, 현재방향, F,G)
bool compare(node a, node b)  { return a.F < b.F; }   //우선순위 queue 조건
bool empty_node=0;
PriorityQueue<node> open_node = PriorityQueue<node>(compare);  // 우선순위 queue 생성

int8_t dirR=4;                          // 현재 로봇방향 ( 1 : 좌,  2 : 하 , 3 : 우 , 4 : 상)
uint8_t stat_x=3, stat_y=0;             // 시작점 좌표
uint8_t dest_x=10, dest_y=0;             // 목적지 좌표
//uint8_t stat_x=6, stat_y=0;             // 시작점 좌표
//uint8_t dest_x=0, dest_y=0;             // 목적지 좌표

unsigned int openF[size_y][size_x];     // F값 비교를 위한 배열
unsigned int Total_G=65000;              // 현 상황에서 출발점-목적지 거리
bool closed[size_y][size_x];            // 닫힌목록  
bool revit=0, complete=0;               // 목적지 도착후 완료 및 재탐색 유무
uint8_t par_x[size_y][size_x], par_y[size_y][size_x];   // 부모노드 저장 배열
uint8_t pos_x,  pos_y;            // 현재 위치
uint8_t pri_x,  pri_y;            // 이전 위치
unsigned int F=0,G=0;             // 누적 F,G
unsigned int unit_node=1250;      // 노드 1칸 엔코더 펄스 
uint8_t cost_d=10, cost_r= 10;     // 이동 cost 

int8_t dirX[4] = { -1, 0, 1, 0 };  // 방향정보 : 좌하우상  
int8_t dirY[4] = { 0, -1, 0, 1 };
 
// 리턴경로 정보
uint8_t ret_x[256],ret_y[256];      // 리턴경로 x,y 좌표
uint8_t ret_rot[256],ret_dis[256];  // 리턴경로 회전 및 거리 정보 
uint8_t rcnt=0, rn=0;               // 리턴경로 노드갯수 및 현재 리턴노드
unsigned int ret_cost=0;            // 리턴경로의 cost (최종거리값)

// OLED Global Variable
uint8_t datad[250][11];       //OLED 표시정보 (방문노드)
uint8_t datas[250][5];        //OLED 표시정보 (최적경로)
uint8_t nns=0;

unsigned int ncnt=0;
   

// 로봇 제어 변수
RPI_PICO_Timer ITimer0(0);

bool vel_dis=0;
int vH=120, vL=60;    //고속,저속 속도
int wL=50;
int vlim=40, wlim=70;
int vd=0, wd=0, dis=0, ang=0;
uint8_t vss=100,sta=0;

uint8_t node=0;                 //현재 노드 번호 = optimum_path()함수의 리턴으로 넣어줌
uint8_t node_end=0;             //끝점 노드 번호
uint8_t node_rot[256], node_dis[256];
uint8_t dv=1;


bool TimerHandler0(struct repeating_timer *t)
{  MM.Get_velo();
   if (vel_dis==0) { MM.Set_velo(vd,wd); }
   else            { MM.Set_dis(dis,ang,vd,wd,10,10); }
   return true;
}

  
void setup() {
  Serial.begin(9600);   delay(500);
  multicore_launch_core1(main2);

  // A* 설정
  pos_x=stat_x; pos_y=stat_y;         // 시작점을 현재위치로
  pri_x=stat_x; pri_y=stat_y; 
  memset(closed,0,sizeof(closed));    // 방문노드 0 리셋
  memset(openF,255,sizeof(openF));    // 오픈노드 F 최대값
  closed[stat_y][stat_x]=1;           // 시작점 방문완료 표시
  
  
  // 로봇 설정
  MM.Motor_Init();
  MM.Set_velo_gain(5,10,5,10,5,150,400);
  MM.Set_dis_gain(1.,0.4,1.,0.4);
  Int_Init();
  ITimer0.attachInterruptInterval(10 * 1000, TimerHandler0);
  
  pinMode(1,INPUT_PULLUP);
  int  p=0;
  while(p<100)  { if (digitalRead(1)==0) {p++;}  else {p=0;}   delay(10);  }   // 버튼 1초 누를때 까지 대기
  vss=10;  p=0;
  while(p<100)  { if (digitalRead(1)==1) {p++;}  else {p=0;}   delay(10);  }   // 버튼 때고 1초 대기
  for (p=2; p>0; p--)   { vss=p; delay(1000); }

  vss=11;  
  
  bool p1=digitalRead(14), p2=digitalRead(15);
  vd=20;
  while (!(p1&&p2))  { p1=digitalRead(14);  p2=digitalRead(15); delay(10);  } 
  vd=vlim; MM.det=2; 
  memset(MM.encCnt,0,sizeof(MM.encCnt)); // 카운터 리셋
 }

void loop() {  
while(1) {
if (sta==0)   // 정상주행
   { MM.cal_pos(800); // 차선정보 업데이트 
     if (MM.det!=2)   // 좌우 차선 중 하나라도 감지된다면 (노드 도착)
      { // 기본 정보 업데이트 
        int node_dis = MM.encCnt[0]+MM.encCnt[1];            //방문노드까지 거리 카운터
        memset(MM.pos,0,sizeof(MM.pos));                     //위치제어 중단
        memset(MM.encCnt,0,sizeof(MM.encCnt));               //카운터 리셋
        int node_num = round( node_dis / float(unit_node));  //방문노드까지 단위거리
        uint8_t pri_x=pos_x, pri_y=pos_y;                    //이전위치
        closed[pos_y + dirY[dirR-1]][pos_x + dirX[dirR-1]] = 1; // 실제 방문하려고 한 점만 closed
        pos_x = pos_x + dirX[dirR-1]*node_num;               //현재위치  
        pos_y = pos_y + dirY[dirR-1]*node_num;
        
        uint8_t next_dir=0;                                  // 다음 이동 방향 (0:막다른길)
        
        G=G + node_num*cost_d;                               // 현재 위치 G값 업데이트  
        F=G + est_dis(pos_x,pos_y,dest_x,dest_y, dirR);      // 현재 위치 F값 업데이트
        if (F < openF[pos_y][pos_x] )                        // 현재 위치 F값이 이전보다 작다면 부모노드 및 F 업데이트
           { par_x[pos_y][pos_x]=pri_x;   par_y[pos_y][pos_x]=pri_y; openF[pos_y][pos_x]=F; 
           }
        
        
        //OLED 정보 업데이트 : 방문노드 넘버, 현재위치
        datad[ncnt][0]=ncnt;  datad[ncnt][1]=pos_x;  datad[ncnt][2]=pos_y;

        // 현재위치 직전까지 방문완료 표시
//        int8_t xs=(pos_x-pri_x>0) - (pos_x-pri_x<0), ys=(pos_y-pri_y>0) - (pos_y-pri_y<0);
//        
//        if (ys==0) {  for (int p=pri_x; p !=pos_x; p=p+xs)  { closed[pos_y][p]=1;  }    }
//        else       {  for (int p=pri_y; p !=pos_y; p=p+ys)  { closed[p][pos_x]=1; }    }    
        

        // 목적지 도착 혹은 재탐색으로 이미 방문한 곳에 도착하였을 때
        if (((pos_x==dest_x)&&(pos_y==dest_y)) || (closed[pos_y][pos_x]&&revit) )  
            { closed[pos_y][pos_x]=1; revit=1;                           // 방문완료 설정후 재탐색
              int tempdir=return_path(dest_x, dest_y, stat_x, stat_y, 0);    
              unsigned int GG=ret_cost; 
              if (GG <  Total_G) { Total_G=GG;}                      // 출발-도착 cost 계산                                        // 출발-도착 cost 계산
              struct node new_node=find_new_node();                      // 방문하지 않은 노드 중 최소 F 노드
              // OLED 정보 업데이트 (추출점 좌표 및 F값)
              datad[ncnt][7]=new_node.x; datad[ncnt][8]=new_node.y;   datad[ncnt][9]=highByte(new_node.F);  datad[ncnt][10]=lowByte(new_node.F);
   
               if (new_node.F < Total_G) // 재탐색
                    { vss=16; G = new_node.G - cost_d;   // 회전만 고려하여 도착점 G값 업데이트
                      dirR=return_path(pos_x, pos_y, new_node.x, new_node.y, dirR);
                      pos_x=par_x[new_node.y][new_node.x];  pos_y=par_y[new_node.y][new_node.x];  }
               else { vss=17; complete=1;   
                      dirR=return_path( pos_x, pos_y, stat_x, stat_y, dirR );  }  // 탐색 완료 : 원점복귀
               next_dir=ret_rot[0]; sta=1;   // 리턴상태로 이동
            }
 
 
        // 일반 노드에서는 탐색
        else 
            { closed[pos_y][pos_x]=1; 
              while(MM.encCnt[0]+MM.encCnt[1]<200) {delay(1); }                 // 좌우 라인이 감지되지 않도록 살짝 전진
              bitWrite(MM.det,1,(analogRead(A1)<300) || (analogRead(A2)<300));  // 직진 라인이 있는지 확인 (좌우라인은 인터럽트로 확인)
              unsigned int temp_F=65535, temp_G=0;  // 탐색에 사용할 임시 F,G값
              
              // 주변 지점 탐색
              for (int p=1; p<4; p++)
              {  if (bitRead(MM.det,3-p)==1)   // 해당 방향으로 길이 존재하면
                    {  int8_t dirU = dirR + 2 - p;          // 해당 방향 이동시 로봇 방향
                       if (dirU>4) {dirU = dirU -4;} else if (dirU<1) {dirU=dirU+4;}
                       uint8_t next_x=pos_x+dirX[dirU-1];   // 탐색점 x좌표
                       uint8_t next_y=pos_y+dirY[dirU-1];   // 탐색점 y좌표
                       datad[ncnt][2+p]=p;     // OLED 정보 업데이트 (회전가능 방향)
                      
                       if ( (closed[next_y][next_x]==0)||(revit) )  // 방문하지 않은 곳 또는 목적지 도착후 재방문이면 거리 예측
                         {   unsigned int g = G + cost_d + (dirU!=dirR)*cost_r;                // 임시 f,g값 (회전이 필요한 경우 코스트 추가)
                             unsigned int f = g + est_dis(next_x,next_y,dest_x,dest_y, dirU);   
      
                             // f값이 이전에 계산된 f보다 작은 경우에만 업데이트
                             if ( f < openF[next_y][next_x] )
                                {  openF[next_y][next_x] = f;
                                   struct node temp={ next_x, next_y, dirU, f, g};
                                   open_node.push(temp);  
                                   par_x[next_y][next_x]=pos_x;  par_y[next_y][next_x]=pos_y; }
                             // 최소 예측거리에 해당하는 정보 저장
                             if (f <= temp_F )  { temp_F=f; temp_G=g; next_dir=p;  }        
                         }               
                    }
              }
              
              // 탐색결과 이동가능한 곳이 있다면, 해당 장소로 위치 업데이트   
              if (next_dir!=0) 
                 {  dirR = dirR + 2 - next_dir;                          // 업데이트 후 로봇 방향 
                    if (dirR>4) {dirR = dirR -4;} else if (dirR<1) {dirR=dirR+4;}
                    G = temp_G-cost_d;                                   // 회전만 고려하여 누적 G값 업데이트
                    // OLED 정보 업데이트 : 실제 회전방향, 목표지점, 예상 F
                    datad[ncnt][6]=next_dir;  datad[ncnt][7]=pos_x+dirX[dirR-1]; 
                    datad[ncnt][8]=pos_y+dirY[dirR-1];   datad[ncnt][9]=highByte(temp_F);  datad[ncnt][10]=lowByte(temp_F);  
                 }
                 
             // 탐색결과 막다른길
             else  
               { struct node new_node=find_new_node();                              // 방문하지 않은 노드 중 최소 F 노드 pop
               
                 // 만일 이 노드의 F가 현 상황에서 목적지까지 거리보다 작다면 이 노드에서 탐색
                 if ( new_node.F < Total_G )                                           
                    {  dirR=return_path(pos_x, pos_y, new_node.x, new_node.y, dirR);   // 해당 노드까지 경로 추출  
                       G = new_node.G - cost_d;                                        // 회전만 고려하여 G값 업데이트
                       pos_x=par_x[new_node.y][new_node.x];                            // 리턴위치 업데이트
                       pos_y=par_y[new_node.y][new_node.x];     } 
                       
                 // 새로운 노드의 F가 현재 목적지까지 거리보다 크다면 탐색 종료      
                 ////////////// 여기서 dis와 rot을 구해야 함 ///////////////////////////////
                 // 여기서 rcnt가 결정
                 else  {  vss=13; complete=1; dirR=return_path(pos_x, pos_y, stat_x, stat_y, dirR);}          

             
                 next_dir=ret_rot[0]; sta=1; // 리턴상태로 이동
 
                 // OLED 정보 업데이트 :  리턴지점, 예상 F
                 datad[ncnt][7]=new_node.x; datad[ncnt][8]=new_node.y;   datad[ncnt][9]=highByte(new_node.F);  datad[ncnt][10]=lowByte(new_node.F);
               }
            }


        // 결정된 정보를 바탕으로 회전유무 선택
        if (next_dir%2!=0)
           {   dis=500; ang=0;  wd=wlim; MM.dis_com=0;  vel_dis=1;  
               while(!MM.dis_com)    { delayMicroseconds(1000); } vel_dis=0; dis=0;  }
  
        memset(MM.pos,0,sizeof(MM.pos));  memset(MM.encCnt,0,sizeof(MM.encCnt)); 
        if      (next_dir==1) { turn(-680); }
        else if (next_dir==2) { vd=vlim; wd=0; }
        else if (next_dir==3) { turn( 680); } 
        else if (next_dir==4) { turn(1300);    } 
        
        ncnt++;  
        dis=0; ang=0;   
        MM.det=0; 
        memset(MM.encCnt,0,sizeof(MM.encCnt));  
        
      } 
   }
else if (sta==1)   // 해당 위치로 리턴 (로봇 이동)
   { MM.cal_pos(800);  if(vss<16){ vss=13;}
     if (MM.det!=2)
     {   if (rn<rcnt-2)
          {  rn++;    // 리턴 노드 넘버 
             memset(MM.pos,0,sizeof(MM.pos));                     // 위치제어 중단
             memset(MM.encCnt,0,sizeof(MM.encCnt));               // 카운터 리셋
             while(MM.encCnt[0]+MM.encCnt[1]<200) {delay(1); }    // 좌우라인에서 살짝 전진
        
             if (ret_rot[rn]%2==1)  //회전이 필요하다면 정의된 방향으로 회전
                { dis=500; ang=0;  wd=wlim; MM.dis_com=0;  vel_dis=1;  
                  while(!MM.dis_com)    { delayMicroseconds(1000); } vel_dis=0;
                  turn(680 *(ret_rot[rn]-2));   }
                  
             MM.det=2; }
         else   //목적지 도착시 상태전환
           { if (complete==0) {sta=0; vd=vlim; wd=0; rn=0; vss=11; } 
             else             {sta=2; vd=0; wd=0; rn=0; node_end = optimum_path(rcnt-1);} }
     }
  }
else if (sta==2)
  { vss=15; // Finish
    MM.Set_velo_gain(5,10,5, 10,5, 200, 800); // 맨 뒤에 두 개가 P, D
    MM.Set_dis_gain(1.,-0.4,1.,-0.4);
    vlim=30;
    // 여기서 버튼을 다시 누르면 최적 경로로 커맨드 주행하도록 추가
    int  p=0;
    while(p<100)  { if (digitalRead(1)==0) {p++;}  else {p=0;}   delay(10);  }   // 버튼 1초 누를때 까지 대기
    vss=10;  p=0;
    while(p<100)  { if (digitalRead(1)==1) {p++;}  else {p=0;}   delay(10);  }   // 버튼 때고 1초 대기
    for (p=2; p>0; p--)   { vss=p; delay(1000); }

    vss=11;  
    
    bool p1=digitalRead(14), p2=digitalRead(15);
    vd=20;
    while (!(p1&&p2))  { p1=digitalRead(14);  p2=digitalRead(15); delay(10);  } 
    vd=vlim; MM.det=2;
    memset(MM.encCnt,0,sizeof(MM.encCnt)); // 카운터 리셋
    
    ///////////// 최적 경로 주행 시작 ////////////
    //////////////// 커맨드 주행 ////////////////
    while(1)
    {
      memset(MM.encCnt,0,sizeof(MM.encCnt));     // 카운터 리셋
      //주행할 노드의 길이가 1보다 큰 경우
       if ( node_dis[node]>1) 
          { 
            if(node_dis[node] > 10){ vH=180; }
            else if(node_dis[node] >2){ vH=140; }
            else{ vH=120; }
            dis=(node_dis[node]-1)*1250 + 450; ang=0;  // 이동할 거리 : 노드수
            memset(MM.encCnt,0,sizeof(MM.encCnt));        // 거리 리셋
            MM.dis_com=0; vel_dis=1;                      // 거리제어
            while(MM.encCnt[0]+MM.encCnt[1]<dis) 
               {MM.cal_pos(850); 
               if(vd<vH){vd = vd + dv;}
               if (MM.det!=2) {MM.det=2; MM.pos[1]=MM.pos[0]; // 이동중 노드 무시
               }   
                delay(1);}   //거리 완료될동안 대기
          }
   
 
      // 마지막 노드가 아니라면 회전
      if (node<node_end)
      {  
         vel_dis=0; vd=vL; wd=0; dis=0; ang=0;
         MM.det=2;  
         while(MM.det==2) {MM.cal_pos(850); delay(1);}      // 노드를 만날때 까지 전진  
         memset(MM.encCnt,0,sizeof(MM.encCnt));             // 카운터 리셋
         memset(MM.pos,0,sizeof(MM.pos));                   // 라인위치 리셋
         while(MM.encCnt[0]+MM.encCnt[1]<400) {delay(1);}// 노드점에서 조금 더 이동
         turn( 700*( node_rot[node]-2 ) );                  // 선택방향으로 회전
         node++;                                            // 노드번호 증가
      }
                     
      // 마지막 노드라면 끝가지 전진
      else { while(MM.det==2) {MM.cal_pos(850); delay(1);}  // 노드 감지때까지 유지
             vd=0; wd=0; sta=99; vss=15; dis=0; ang=0;}     // 목적지 도착     
    }

      
  }  
delayMicroseconds(100);             
}     
}


void main2()    // OLED 표시
{  bool dps=0;  // 0:방문노드 출력 1:최적경로 출력
   bool chg=1;
   int8_t cnt=0, c1=0, dcnt;
   String q1,q2;
   
   MM.OLED_Init();
   while(1) 
   { 
     if (vss>10)
      { if (digitalRead(1)==0) 
           {cnt++; if ((cnt>10)&&chg) {c1++; cnt=0; chg=0; if (c1>dcnt-1) {c1=0;} } }
        else {cnt=0; chg=1;}  }

    if (dps==0)
      { dcnt=ncnt ;
        String dik;
        for (int p=3; p<=5; p++)
        { if      ( datad[c1][6]==p-2) {dik=dik+"O"; }
          else if ( datad[c1][p] >  0) {dik=dik+"o"; }
          else                         {dik=dik+"_"; }    }
    
        q1=String(datad[c1][0])+" "+String(datad[c1][1])+","+String(datad[c1][2])+" "+dik;
        q2=String(dcnt)+" "+String(datad[c1][7])+","+String(datad[c1][8])+" "+String(word(datad[c1][9],datad[c1][10])); }
     
    else 
       { dcnt=rcnt ;
         q1=String(c1)+ " (" + String(datas[c1][0]) + "," + String(datas[c1][1]) + ")"  ;
         //q2=String(dcnt)+ "  R" + String(datas[c1][2]) + "," + " D" + String(datas[c1][3]);
         q2=String(dcnt)+ " (" + String(datas[c1+1][0]) + "," + String(datas[c1+1][1]) + ")"  ;
       }
    
    MM.OLED_D0(vss);  MM.OLED_D1(q1);   MM.OLED_D2(q2);  MM.OLED_Disp();  
   delay(20); } 
 
}



// 회전함수
void turn (int ad)
{     memset(MM.encCnt,0,sizeof(MM.encCnt));   
      ang=ad; dis=0; vel_dis=1;
      wd=wlim; vd=vlim;  MM.dis_com=0; 
      bool p1=0;
      unsigned int count=0;
      
      while(! (MM.dis_com || p1)) 
      { p1=( (abs( MM.encCnt[0]-MM.encCnt[1] - ang )< 100 ) && ((analogRead(A1)<250) && (analogRead(A2)<250))); 
        delayMicroseconds(1000);
        if(count>10){if(wd>wL){wd = wd - 1;} count=0;}
        count++;
        }
      
       vel_dis=0; vd= 0; wd=0;
       memset(MM.pos,0,sizeof(MM.pos));  
       MM.cal_pos(850); 
       while(abs(MM.pos[1])>1)  { MM.cal_pos(850); delay(1);  }  
       memset(MM.pos,0,sizeof(MM.pos));  
      
       wd=0; vd=vlim; ang=0;
}

 
// A* 거리 예측
unsigned int est_dis(int px, int py, int dx, int dy, uint8_t dir  )
{ int xk=dx-px,  yk=dy-py;
  unsigned int d=cost_r;   
  if      ((xk>0)-(xk<0) + 2 == dir )     { d=0; } // 초기회전이 필요하지 않는다면, 값을 0으로 초기화.
  else if ((yk>0)-(yk<0) + 3 == dir )     { d=0; } 
  else if ((xk==0)&&(yk==0))              { d=0; } // 목적지라면 회전 0

  d = d + cost_d*(abs(xk) + abs(yk)) ;
  if ( (xk != 0) && (yk != 0) ) { d=d+cost_r;  }    // 중간 경로에서 회전해야 하는 경우 회전 추가
   
   return d;  }


// 리턴경로 계산
uint8_t return_path(uint8_t px, uint8_t py, uint8_t dx, uint8_t dy, uint8_t dir)
 {    uint8_t rx[200]={dx}, ry[200]={dy};    // 리턴지점 부모노드
      unsigned int p1=0, p2=0, k=1;          // 부모노드갯수, 공통부분 갯수

        memset(ret_x,0,sizeof(ret_x));memset(ret_y,0,sizeof(ret_y)); 
        memset(ret_rot,0,sizeof(ret_rot)); memset(ret_dis,0,sizeof(ret_dis));
      // 현재위치 부모노드
      ret_x[0]=px;  ret_y[0]=py;
      while ( (px!=stat_x)||(py!=stat_y) )
      {  p1++; 
         uint8_t temp_x=par_x[py][px], temp_y=par_y[py][px]; 
         ret_x[p1]=temp_x; ret_y[p1]=temp_y;
         px=temp_x; py=temp_y;    }


      // 리턴지점이 출발점이 아닌 경우  
      if ((dx!=stat_x)||(dy!=stat_y))  
        {   
           // 리턴지점 부모노드
           while ( (dx!=stat_x)||(dy!=stat_y) )
             {  p2++; 
                uint8_t temp_x=par_x[dy][dx], temp_y=par_y[dy][dx]; 
                rx[p2]=temp_x; ry[p2]=temp_y;
                dx=temp_x; dy=temp_y;    } 

           while(!((ret_x[p1-k]!=rx[p2-k])||(ret_y[p1-k]!=ry[p2-k])))  { k++; }                      // 공통부분 감지
           for (int i=0; i<=p2-k; i++)  { ret_x[p1-k+2+i]=rx[p2-k-i]; ret_y[p1-k+2+i]=ry[p2-k-i]; }  // 공통부분 제외 후 합치기                               
           rcnt=p1+p2-2*k+3;  }      // 방문할 노드의 갯수
      else
        { rcnt=p1+1;   }                      
 
        
      ret_cost=0;           // 코스트값 리셋
      // 리턴 회전정보 및 코스트값 계산
      for (int p=0; p<rcnt-1; p++)
      {  
         int8_t xk=ret_x[p+1]-ret_x[p],  yk=ret_y[p+1]-ret_y[p];  // 다음점과의 거리차이
         
         uint8_t dir2= 2*(yk>0) +  (xk>0)-(xk<0) + 2;             // 다음점 이동후 로봇방향
         if (dir==0) {dir=dir2;}                                  // 방향 0 입력시, 초기회전 고려하지 않음
         int8_t r21=dir-dir2+2;                                   // 다음점으로 가기위한 회전유무 (1좌2직3우4U)
         if (r21<1) {r21=r21+4;}  else if (r21>4) {r21=r21-4;}

         ret_cost = ret_cost + cost_d*( abs(xk)+abs(yk) ) + (dir!=dir2)*cost_r;
         ret_rot[p]=r21;
         ret_dis[p] = abs(xk) + abs(yk);
         dir=dir2;
       
      }
      
      return dir;   // 리턴 완료 후 로봇방향
}



//방문하지 않은 새로운 노드 찾기
struct node find_new_node()
{   bool done=0;
    struct node bn;
    while(!done)
      { bn=open_node.pop();       // 최소 F값 노드 Pop
        if (closed[bn.y][bn.x]==0) {done=1; }  }
            
    return bn;     
}

 
// 최적경로
// cnt는 부모노드 개수가 아니라 rot, dis 개수
uint8_t optimum_path(uint8_t cnt) 
{ 
  memset(node_rot,0,sizeof(node_rot)); memset(node_dis,0,sizeof(node_dis));
  // cnt는 rot, dis의 길이
  // 이때 직진이 포함되어 있는 rot, dis의 길이임
  // optimum_path가 직진이 포함된 rot, dis의 직을 없앰
  uint8_t i=0;
  while(cnt>0)
  {
    while(ret_rot[cnt-1]==2 && cnt-2>=0){ret_dis[cnt-2] += ret_dis[cnt-1]; cnt--;}
    node_dis[i]=ret_dis[cnt-1];
    if(ret_rot[cnt-1]==1){node_rot[i]=3;} // 방향 반대
    else if(ret_rot[cnt-1]==3){node_rot[i]=1;}
    cnt--; i++;
  }
  return i;
}