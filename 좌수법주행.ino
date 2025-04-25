#include <micromouse.h>
#include <variable.h>
#include <RPi_Pico_TimerInterrupt.h>

RPI_PICO_Timer ITimer0(0);

enum Sta : uint8_t {
    EXPLORE = 0,  // 미로 탐색
    WAIT = 1,     // 재시작 버튼 대기 모드
    DRIVE  = 2,   // 막다른 길 빼고 주행
};

bool vel_dis=0;
int vlim=40, wlim=40;
int vd=0, wd=0, dis=0, ang=0;

int kpv = 10;
int kdv = 3;

int kpw = 10;
int kdw = 3;

uint8_t vss=100;
Sta sta = EXPLORE; // 초기 상태

// EXPLORE
int mz[50], mz_cnt=0; // 방향 기록
int mz_sgn = 0;
int el[50], el_cnt;

// DRIVE
int ncnt = 0;
int dir = 0;

bool TimerHandler0(struct repeating_timer *t)
{  MM.Get_velo();
   if (vel_dis==0) { MM.Set_velo(vd,wd); }
   else            { MM.Set_dis(dis,ang,vd,wd,10,10); }
   return true;
}

// init
void setup() {
  Serial.begin(9600);   delay(500);
  multicore_launch_core1(main2);
  
  MM.Motor_Init();
  MM.Set_velo_gain(5,kpv,kdv,kpw,kdw,100,20);
  MM.Set_dis_gain(1.,0.4,1.,0.4);
  Int_Init();
  ITimer0.attachInterruptInterval(10 * 1000, TimerHandler0);
  
  pinMode(1,INPUT_PULLUP);
  int  p=0;
  while(p<100)  { if (digitalRead(1)==0) {p++;}  else {p=0;}   delay(10);  }   // 버튼 1초 누를때 까지 대기
  vss=10;  p=0; // Ready
  while(p<100)  { if (digitalRead(1)==1) {p++;}  else {p=0;}   delay(10);  }   // 버튼 때고 1초 대기

  for (p=2; p>0; p--)   { vss=p; delay(1000); }

  vss=12;  // Drive
  
  bool p1=digitalRead(14), p2=digitalRead(15);
  vd=20;
  while (!(p1&&p2))  { p1=digitalRead(14);  p2=digitalRead(15); delay(10); } 
  vd=vlim; MM.det=2; 

}

// main loop
void loop() {
    switch (sta) {
      case EXPLORE:   handleExplore();   break;
      case WAIT:      handleWait();   break;
      case DRIVE:     handleDrive();    break;
    }
}

// 미로 탐색
void handleExplore() {
    '''
    [MM.det]
    bit0=1 -> 오른쪽 길 감지
    bit1=2 -> 직진 길 감지
    bit2=4 -> 왼쪽 길 감지
    '''
    // 라인 트레이싱
    MM.cal_pos(800);
    
    // EVENT : 갈림길
    if (MM.det != 2) {
        el[el_cnt] = (MM.encCnt[0] + MM.encCnt[1]) / 1250; // 엔코더 to 칸 -> 갈림길 전까지 이동칸 기록
        Serial.println(el[el_cnt]);
        
        decision(440); // 앞으로 조금 전진하여 판단

        if (mz_sgn==0) {mz_cnt++; } 
        else {mz_cnt--;} // 막다른길에서 되돌아 오는 중
        
        if      (MM.det>=8) { sta=WAIT; vss=15; vd=0; wd=0;}  // 도착점
        else if (MM.det>=4) { turn2(-40);    mzsol(1);   }       // 왼쪽회전
        else if (MM.det>=2) { vd=40; wd=0;   mzsol(2);   }       // 직진
        else if (MM.det>=1) { turn2( 40);    mzsol(3); }         // 오른쪽 회전
        else                { turn2(-40);    mz_sgn = 1;  }      // 막다른길
        
        MM.det=2;wd=0;
        memset(MM.encCnt,0,sizeof(MM.encCnt));
        el_cnt++;
    }
}

// 재시작 버튼 대기 모드
void handleWait(){
    int  p=0;
    while(p<100)  { if (digitalRead(1)==0) {p++;}  else {p=0;}   delay(10);  }   // 버튼 1초 누를때 까지 대기
    vss=10;  p=0; // Ready
    while(p<100)  { if (digitalRead(1)==1) {p++;}  else {p=0;}   delay(10);  }   // 버튼 때고 1초 대기
    vss= 12; 
    
    bool p1=digitalRead(14), p2=digitalRead(15);
    vd=20;
    while (!(p1&&p2))  { p1=digitalRead(14);  p2=digitalRead(15); delay(10); } 
    vd=vlim; MM.det=2; sta = 2; 
}

// 막다른 길 빼고 주행
void handleDrive(){
    // 라인 트레이싱
    MM.cal_pos(800);

    // 갈림길
    if (MM.det!=2) { 
      ncnt++;    dir=mz[ncnt]; vss=ncnt;
      memset(MM.pos,0,sizeof(MM.pos));
      memset(MM.encCnt,0,sizeof(MM.encCnt));
      
      // 직진이 아니면
      if (dir!=2){
        dis=400; ang=0;  wd=wlim; MM.dis_com=0;  vel_dis=1;  
        while(!MM.dis_com)    { delayMicroseconds(1000); }
      }
      
      if ((ncnt==mz_cnt)||(dir==0)) {vd=0; wd=0; sta=3; vss=15;} // 도착
      else if (dir==1) {turn2(-40);  }
      else if (dir==2) {while(MM.encCnt[0]+MM.encCnt[1]<300) {delay(1); }   }
      else if (dir==3) {turn2( 40);  }
      MM.det=0; dis=0; ang=0; vel_dis=0;   
    }
}


void decision(int dd)
{   
    memset(MM.pos,0,sizeof(MM.pos)); // position값 리셋, 속도 제어를 안하기 위함
    memset(MM.encCnt,0,sizeof(MM.encCnt)); // 엔코더값 리셋, 0부터 원하는 값을 측정하기 위함
    delay(50);
    bitWrite(MM.det,1,(analogRead(A1)<300) || (analogRead(A2)<300)); 
    bitWrite(MM.det,3,!(digitalRead(14) || digitalRead(15)) ); 
    if ((MM.det!=2) && (MM.det!=3))
       {   dis=dd; ang=0;  wd=wlim; MM.dis_com=0;  vel_dis=1;  
           while(!MM.dis_com)    { delayMicroseconds(1000); }  vel_dis=0; }
           wd=0; dis=0; 
}

void turn (int ad) { 
  // 엔코더 초기화 => 지금부터 모터 회전각    
  memset(MM.encCnt,0,sizeof(MM.encCnt));   
  ang=ad; dis=0;
  MM.dis_com=0; 
  bool p1=0;
  while(! (MM.dis_com || p1)) 
  { p1=(abs(MM.ome[1] - ad/2)< 100 ) && ((analogRead(A1)<250) && (analogRead(A2)<250)); 
    delayMicroseconds(1000); }

    vel_dis=0; wd=0;   vd=vlim;   
}

void turn2 (int wa) {  
   vd=0; wd=wa; vel_dis=0; dis=0; ang=0; MM.dis_com=0;
   int f1=analogRead(A1), f2=analogRead(A2); 
   while ((f1<300)||(f2<300)) 
     { f1=analogRead(A1); f2=analogRead(A2); delayMicroseconds(1000);  }
   while (!(f1<250)&&!(f2<250))
     { f1=analogRead(A1); f2=analogRead(A2); delayMicroseconds(1000);  } 
   wd=0;   vd=vlim; 
}

void mzsol(int p) 
{
  if(mz_sgn == 0){mz[mz_cnt] = p;} // 정상 -> 방향 기록
  else{
    mz[mz_cnt] = mz[mz_cnt] + p;  // 막다른길에서 돌아온 후 마주한 갈림길
    if(mz[mz_cnt] > 3) {mz[mz_cnt] = 0;} // 0 => 막다른 경로들
    else {mz_sgn=0;} // 막다르지 않은 길일 때에만 mz_sgn을 탈출할 수 있음
  }
}


// OLED
void main2()
{   
   MM.OLED_Init();
   while(1) 
   { MM.OLED_D0(vss); 
  //     if (mz_cnt<12)  { MM.OLED_D1(MM.arr2str(mz, 0 ,mz_cnt) ); }
  //     else  { MM.OLED_D1(MM.arr2str(mz, 0 ,11) );
  //             MM.OLED_D2(MM.arr2str(mz, 12 ,mz_cnt) ); 
    if(el_cnt < 12) { MM.OLED_D1(MM.arr2str(el, 0, el_cnt)); }
    else            {MM.OLED_D1(MM.arr2str(el, 0 ,11) );
                     MM.OLED_D2(MM.arr2str(el, 12 ,el_cnt));}

     MM.OLED_Disp();  
     delay(20);}
}