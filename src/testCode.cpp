

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include  <IRremote2.h>
#include <LiquidCrystal.h>

/* ---------- TRIS VARIABLE ----------  */
#define INT_MIN  -2147483648
#define INT_MAX  +2147483647
char Tris[3][3];
int  TrisPos[3][3][4] =   {{{-55,280,70,105},{0,280,60,90},{65,280,70,75}}, //x,y,z,x5    {0,0},{0,1},{0,2}
                          {{-60,210,50,110},{0,215,50,90},{60,220,50,70}},  //            {1,0},{1,1},{1,2}
                          {{-65,150,45,115},{0,155,45,90},{55,160,45,65}}  //            {2,0},{2,1},{2,2}
                         };
int TakePos[6][4]  = {{20,-140,40,100},{15,-205,60,95},{10,-240,90,90},{-75,-230,80,75},{-70,-175,70,70},{-65,-120,50,65}};
int  pawn=0;
int choice;

/* ---------- IR VARIABLE ---------- */
// RECV_PIN  = 52;
IRrecv irrecv(52);
decode_results results;

/* ---------- LIQUIDCRYSTAL  VARIABLE ---------- */
// pin lcd: rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7  = 7;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

/* ---------- ROBOTIC ARM VARIABLE  ---------- */
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define  BASE_HGT 115 //base height
/*#define HUMERUS  105   shoulder-to-elbow "bone"
  #define ULNA     100   elbow-to-wrist    "bone"
  #define GRIPPER  170   wrist-to-gripper  "bone"   */
#define SERVO1  1
#define SERVO2  2
#define SERVO3  3
#define  SERVO4  4
#define SERVO5  5
#define SERVO6  6

#define SERVOMIN1  130
#define  SERVOMAX1  430
#define SERVOMIN2  120
#define SERVOMAX2  365
#define SERVOMIN3  150
#define SERVOMAX3  430
#define SERVOMIN4  130
#define SERVOMAX4  435
#define  SERVOMIN5  100
#define SERVOMAX5  380
#define SERVOMIN6  200
#define SERVOMAX6  325
unsigned int ServoMinMax[6][2] = {{SERVOMIN1,SERVOMAX1},
                                  {SERVOMIN2,SERVOMAX2},
                                  {SERVOMIN3,SERVOMAX3},
                                  {SERVOMIN4,SERVOMAX4},
                                  {SERVOMIN5,SERVOMAX5},
                                  {SERVOMIN6,SERVOMAX6}};

float  x1,x2,x3,x4,x5,x6 = 90;                          // servo position
float oldx1,oldx2,oldx3,oldx4,oldx5,oldx6  = x1;        // old servo position

float xaxis =   0;
float yaxis = 150;
float  zaxis = 100;

bool checkmin = true;
bool checkmax = true;

/* ----------  setup ---------- */
void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0));
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print(">>> T R I S <<<");
  lcd.setCursor(0,1);
  lcd.print("   by Danny003");
  delay(2000);
  pwm.begin();
  pwm.setPWMFreq(50);
  MyServoWrite(SERVO2,90);
  delay(1000);
  Set_Arm(xaxis,yaxis,zaxis);
  irrecv.enableIRIn();
}

/* ---------- loop ---------- */
void loop()  {
  choice = 0;
  for(int i=0;i<3;i++) {
    for(int j=0;j<3;j++) { Tris[i][j]='  '; }
  }
  pawn = 0;  
  lcd.clear();
  lcd.print("Choose Who Start");
  lcd.setCursor(0,1);
  lcd.print("CH-=Comp CH+=You");
  choice=Read_RC();  
  while(choice==10 )  {
   Computer_Move();
    if(Check_Win('O')) {
      Show_Win(Check_Win('O'));
      Set_Arm( xaxis = 0.0, yaxis = 130.0 , zaxis  = 200.0 );
      lcd.clear();
      lcd.print("    Computer  ");
      lcd.setCursor(0,1);
      lcd.print("    W I N S   ");
      delay(2000);
      break;
    }
    if(End_Game()) {
      lcd.clear();
      lcd.print("  Nobody wins ");
      lcd.setCursor(0,1);
      lcd.print("    F L A P   ");
      delay(2000);
      break;
    }
    Player_Move();
    if(Check_Win('X')) {
      lcd.clear();
      lcd.print("    Player  ");
      lcd.setCursor(0,1);
      lcd.print("    W I N S   ");
      delay(2000);
      break;
    }
    if(End_Game())  {
      lcd.clear();
      lcd.print("  Nobody wins ");
      lcd.setCursor(0,1);
      lcd.print("    F L A P   ");
      delay(2000);
      break;
    }
  }
  while(choice==11)  {
    Player_Move();
    if(Check_Win('X')) {
      lcd.clear();
      lcd.print("    Player  ");
      lcd.setCursor(0,1);
      lcd.print("    W I N S   ");
      delay(2000);
      break;
    }
    if(End_Game()) {
      lcd.clear();
      lcd.print("  Nobody wins ");
      lcd.setCursor(0,1);
      lcd.print("    F L A P   ");
      delay(2000);
      break;
    }
   Computer_Move();
    if(Check_Win('O')) {
      Show_Win(Check_Win('O'));
      Set_Arm( xaxis = 0.0, yaxis = 130.0 , zaxis = 200.0 );
      lcd.clear();
      lcd.print("    Computer  "); 
      lcd.setCursor(0,1);
      lcd.print("    W I N S   ");
      delay(2000);
      break;
    }    
    if(End_Game())  {
      lcd.clear();
      lcd.print("  Nobody wins ");
      lcd.setCursor(0,1);
      lcd.print("    F L A P   ");
      delay(2000);
      break;
    }
  }
  Tidy();
}


/* -------------------- ROBOTIC ARM FUNCTION --------------------  */

/* ---------- MyServoWrite ---------- */
/* This funciont is used to  move a singular servo*/
void MyServoWrite(int servo, float gradi) {
     pwm.setPWM(  servo, 0, MapNew(gradi, 0.0, 180.0, ServoMinMax[servo-1][0], ServoMinMax[servo-1][1]  ));
}

/* ---------- MyServoWriteAll ---------- */
/* This funciont  is used to move the first 4 servo, it has been created to avoid to repeat the funciont  MyServoWrite 4 times */
void MyServoWriteAll(float y1,float y2,float y3,float  y4) {
  pwm.setPWM( SERVO1, 0, MapNew(y1, 0.0, 180.0, ServoMinMax[SERVO1-1][0],  ServoMinMax[SERVO1-1][1] ));
  pwm.setPWM( SERVO2, 0, MapNew(y2, 0.0, 180.0,  ServoMinMax[SERVO2-1][0], ServoMinMax[SERVO2-1][1] ));
  pwm.setPWM( SERVO3,  0, MapNew(y3, 0.0, 180.0, ServoMinMax[SERVO3-1][0], ServoMinMax[SERVO3-1][1] ));
  pwm.setPWM( SERVO4, 0, MapNew(y4, 0.0, 180.0, ServoMinMax[SERVO4-1][0], ServoMinMax[SERVO4-1][1]  ));
}

/* ---------- MyServoWriteGradual ---------- */
/*This function  is similar to MyServoWriteAll; but to avoid too rapid movement, 
 in particular  when the robotic arm goes from a position to another one,this function creates 
  a smooth and simultaneous movement of the 4 servo, whose time depends on the variation  of the bigger angle*/
void MyServoWriteGradual( float newpos1, float newpos2,  float newpos3, float newpos4 ) {
  float oldpos1 = oldx1;
  float oldpos2  = oldx2;
  float oldpos3 = oldx3;
  float oldpos4 = oldx4;
  float differ1  = newpos1 - oldpos1;
  float differ2 = newpos2 - oldpos2;
  float differ3  = newpos3 - oldpos3;
  float differ4 = newpos4 - oldpos4;
  float largest  = max(abs(differ1),max(abs(differ2), max(abs(differ3),abs(differ4))));
  for  ( int i = 0; i <= largest; i++ ) {
     oldpos1 += (differ1/largest);
     oldpos2  += (differ2/largest);
     oldpos3 += (differ3/largest);
     oldpos4 += (differ4/largest);
     MyServoWriteAll(oldpos1,oldpos2,oldpos3,oldpos4);
     delay(10);       
  } 
  MyServoWriteAll(newpos1,newpos2,newpos3,newpos4);
} 
 
/* ----------  Set_Arm ---------- */
/* This function is the most important for the movement  of the robotic arm, 
   because it calculates the postions of the first 4 servo  from the three coordinates x,y,z(taken from TakePos and TrisPos).
   The following  code is specific to the configuration of MY robotic arm, so if you want to use it,  you have to adapt it to your arm.*/
void Set_Arm( float x, float y, float z)  {
  oldx1 = x1;
  oldx2 = x2;
  oldx3 = x3;
  oldx4 = x4;
  z=z-BASE_HGT;
  float dist_y_z = sqrt( sq(z) +  sq(x) + sq(y) ); 
  if ( dist_y_z < 120.0 )  { checkmin=false; } 
    else { checkmin=true; }
  if ( dist_y_z > 370.0 )  { checkmax=false; } 
    else { checkmax=true; }
  if ( checkmin && checkmax  ) {
    float alfa = 180.0-(degrees(acos((-27500 + sqrt( sq(27500) - 71400*(14225  - sq(dist_y_z))))/71400)));
    float gamma = degrees(acos((27875 -sq(dist_y_z)  - (34000*cos(radians(alfa))))/(-210*dist_y_z)));
    float omega = degrees(acos(sqrt(sq(x)+sq(y))/dist_y_z));
    x1 = 90.0-degrees(atan2(-x,abs(y)));
    if ( z > 0.0 ) { x2=180.0-(gamma+omega);  }
      else { x2=180.0-(gamma-omega); }  
    if ( y < 0.0) { x3=x4=(alfa-90.0);  }
      else {
        x3=x4=(270.0-alfa);
        x2=180.0-x2;
        x1=180.0-x1;
      }
    if ((abs(oldx1-x1)+abs(oldx2-x2)+abs(oldx3-x3)+abs(oldx4-x4)) > 20.0)  { MyServoWriteGradual(x1,x2,x3,x4); }
      else { MyServoWriteAll(x1,x2,x3,x4);  }
  } 
}

/* ---------- Reach_Position ---------- */
/*Through this  function, the Robotic Arm grabs a pawn  from a position and it leaves the pawn in  another position  */
void Reach_Position ( float fromx, float fromy, float fromz,  float fromangle, float tox, float toy, float toz, float toangle ) {
  Set_Arm(  fromx, fromy, fromz+50 );
  delay(300);
  MyServoWrite( SERVO6, 0 );
  MyServoWrite(  SERVO5, fromangle );
  delay(1000);
  Set_Arm( fromx, fromy, fromz );
  delay(300);
  MyServoWrite( SERVO6, 130 );
  delay(500);
  Set_Arm( fromx,  fromy, fromz + 100 );
  delay(500);
  Set_Arm( tox, toy, toz+150);
  delay(1000);
  MyServoWrite( SERVO5, toangle );
  delay(300);
  Set_Arm( tox, toy, toz  );
  delay(300);
  MyServoWrite( SERVO6, 0 );
  delay(1000);
  Set_Arm(  xaxis = 0.0, yaxis = 130.0 , zaxis = 200.0 );
  MyServoWrite( SERVO5, 90 );
  }

/* ---------- MapNew ---------- */
/*This function has been created  because the Arduino map() has not the funciont  constrain() and it does not return  a int value */
int MapNew( float x, float in_min, float in_max, float out_min,  float out_max ) {
   return round((constrain(x,in_min,in_max) - in_min) * (out_max  - out_min) / (in_max - in_min) + out_min);
}
 
/* ---------- Show_Win ----------  */
/*Through this function, if computer wins, the Robotic Arm moves over the  tris to show to the player that it have just won */ 
void Show_Win(int triscase)  {
  MyServoWrite(SERVO6,150);
  switch(triscase){
    case 1:
      Set_Arm(  TrisPos[0][0][0],TrisPos[0][0][1],TrisPos[0][0][2]+100);
      Set_Arm( TrisPos[2][2][0],TrisPos[2][2][1],TrisPos[2][2][2]+100);
      break ;
    case 2:
      Set_Arm( TrisPos[2][0][0],TrisPos[2][0][1],TrisPos[2][0][2]+100);
      Set_Arm( TrisPos[0][2][0],TrisPos[0][2][1],TrisPos[0][2][2]+100);
      break  ;
    case 3:
      Set_Arm( TrisPos[0][0][0],TrisPos[0][0][1],TrisPos[0][0][2]+100);
      Set_Arm( TrisPos[0][2][0],TrisPos[0][2][1],TrisPos[0][2][2]+100);
      break  ;
    case 4:
      Set_Arm( TrisPos[1][0][0],TrisPos[1][0][1],TrisPos[1][0][2]+100);
      Set_Arm( TrisPos[1][2][0],TrisPos[1][2][1],TrisPos[1][2][2]+100);
      break  ;
    case 5:
      Set_Arm( TrisPos[2][0][0],TrisPos[2][0][1],TrisPos[2][0][2]+100);
      Set_Arm( TrisPos[2][2][0],TrisPos[2][2][1],TrisPos[2][2][2]+100);
      break  ;
    case 6:
      Set_Arm( TrisPos[0][0][0],TrisPos[0][0][1],TrisPos[0][0][2]+100);
      Set_Arm( TrisPos[2][0][0],TrisPos[2][0][1],TrisPos[2][0][2]+100);
      break  ;
    case 7:
      Set_Arm( TrisPos[0][1][0],TrisPos[0][1][1],TrisPos[0][1][2]+100);
      Set_Arm( TrisPos[2][1][0],TrisPos[2][1][1],TrisPos[2][1][2]+100);
      break  ;
    case 8:
      Set_Arm( TrisPos[0][2][0],TrisPos[0][2][1],TrisPos[0][2][2]+100);
      Set_Arm( TrisPos[2][2][0],TrisPos[2][2][1],TrisPos[2][2][2]+100);
      break  ;
      
  }
  MyServoWrite(SERVO6,0);
}


/* --------------------  TIC TAC TOE FUNCTION -------------------- */
//The following functions are taken  from : http://www.pierotofy.it/pages/sorgenti/browse/17648/3447/ 

/* ----------  Possible ---------- */
/*This function check if position i,j is free. */
bool  Possible(int i, int j) { return Tris[i][j]==' '; }

/* ---------- End_Game  ---------- */
/*This function check if all position are occupied by a pawn. */
bool  End_Game(){
  for(int i=0;i<3;i++) {
    for(int j=0;j<3;j++) {
      if(Possible(i,j)){return  false;}
    }
  }
  return true;
} 

/* ---------- Check_Win ----------  */
/*This function check all the 8 possible combinations to win at tris. */
int  Check_Win(char wnr) {
  int returnvalue=0;
  if(Tris[0][0]==wnr&&Tris[1][1]==wnr&&Tris[2][2]==wnr)  {returnvalue = 1;}
  if(Tris[2][0]==wnr&&Tris[1][1]==wnr&&Tris[0][2]==wnr) {returnvalue  = 2;}
  if(Tris[0][0]==wnr&&Tris[0][1]==wnr&&Tris[0][2]==wnr) {returnvalue =  3;}
  if(Tris[1][0]==wnr&&Tris[1][1]==wnr&&Tris[1][2]==wnr) {returnvalue = 4;}
  if(Tris[2][0]==wnr&&Tris[2][1]==wnr&&Tris[2][2]==wnr) {returnvalue = 5;}
  if(Tris[0][0]==wnr&&Tris[1][0]==wnr&&Tris[2][0]==wnr)  {returnvalue = 6;}
  if(Tris[0][1]==wnr&&Tris[1][1]==wnr&&Tris[2][1]==wnr) {returnvalue  = 7;}
  if(Tris[0][2]==wnr&&Tris[1][2]==wnr&&Tris[2][2]==wnr) {returnvalue =  8;}
  return returnvalue;
}

/* ---------- Computer_Move ----------  */
/* This function consider which position is better and move the robotic arm  */
void Computer_Move() {
  long maxvalue=INT_MIN,mi=1,mj=1,t;
  lcd.clear();
  lcd.print(" I Am Thinking");
  lcd.setCursor(0,1);
  lcd.print("    Wait");
  if( choice == 10 && pawn == 0 ) {
    mi = int(random(0,3));
    mj = int(random(0,3));
  }
  else {  
    for(int i=0;i<3;i++) {
      for(int j=0;j<3;j++) {
        if(Possible(i,j)) {
          Tris[i][j]='O';
          t=Consider_Move('X',  20);
          if(t>maxvalue) {
            maxvalue=t;
            mi=i;
            mj=j;
          }
          Tris[i][j]=' ';
        }
      }
    } 
  }      
  Tris[mi][mj]='O';
  Reach_Position(TakePos[pawn][0],TakePos[pawn][1],TakePos[pawn][2],TakePos[pawn][3],TrisPos[mi][mj][0],TrisPos[mi][mj][1],TrisPos[mi][mj][2],TrisPos[mi][mj][3]);
  pawn++;
}

/* ---------- Consider_Move ---------- */
/* This function  uses minimax algoritm */
long Consider_Move(char wnr, int deep) {
  int i,  j, res, tmp;
  if(Check_Win('O')) { return INT_MAX; }
  if(End_Game())     {  return 0; }
  if(wnr=='X') {
    res=1;
    for(i=0;i<3;i++) {
      for(j=0;j<3;j++)  {
        if(Possible(i,j)) {
          Tris[i][j]='X';
          if(Check_Win('X'))  {
            if(deep==20) {
              Tris[i][j]=' ';
              return  INT_MIN;
            }
            else {res-=2;}
          }
          else  if((tmp=Consider_Move('O', deep - 1))<res) { res=tmp; }
          Tris[i][j]='  ';
        } 
      }
    }  
  }
  else {
    res=-1;
    for(i=0;i<3;i++)  {
      for(j=0;j<3;j++) {
        if(Possible(i,j)) {
          Tris[i][j]='O';
          if(Check_Win('O')) {res+=2;}
          else if((tmp=Consider_Move('X',  deep - 1))>res) {res=tmp;}
          Tris[i][j]=' ';
        }
      }
    }
  }
  return res;
}

/* ---------- Player_Move ---------- */
/*  This function, depending on which button you pressed, update the position of your  pawns */
void Player_Move() {
  int i=0;
  lcd.clear();
  lcd.print("  Insert your   ");
  lcd.setCursor(0,1);
  lcd.print("    M O V E     ");
  do {
    i=Read_RC();
    if      ((i==9)&&Possible(0,0)) { Tris[0][0]='X';  } 
    else if ((i==8)&&Possible(0,1)) { Tris[0][1]='X'; }   
    else if  ((i==7)&&Possible(0,2)) { Tris[0][2]='X'; }  
    else if ((i==6)&&Possible(1,0))  { Tris[1][0]='X'; }  
    else if ((i==5)&&Possible(1,1)) { Tris[1][1]='X'; }  
    else if ((i==4)&&Possible(1,2)) { Tris[1][2]='X'; }  
    else if ((i==3)&&Possible(2,0))  { Tris[2][0]='X'; }  
    else if ((i==2)&&Possible(2,1)) { Tris[2][1]='X'; }  
    else if ((i==1)&&Possible(2,2)) { Tris[2][2]='X'; }   
    else {
      i=0;
      lcd.clear();
      lcd.print(" I N V A L I D  ");
      lcd.setCursor(0,1);
      lcd.print("    M O V E     ");}
  }while(i<1||i>9); 
}

/* ----------  Read_RC ---------- */
/* This function read the button you pressed  */
int  Read_RC(){
  int returnvalue=99;
  while (returnvalue==99){
    while (!irrecv.decode(&results))  {};
    switch(results.value){
      case 0xFFA25D: // CH-
      returnvalue=10;
      break;
      case 0xFFE21D: //CH+
      returnvalue=11;
      break;
      case 0xFF30CF: // 1
      returnvalue=1;
      break ;
      case  0xFF18E7: // 2
      returnvalue=2;
      break ;
      case 0xFF7A85:  // 3
      returnvalue=3;
      break ;
      case 0xFF10EF: // 4
      returnvalue=4;
      break ;
      case 0xFF38C7: // 5
      returnvalue=5;
      break  ;
      case 0xFF5AA5: // 6
      returnvalue=6;
      break ;
      case  0xFF42BD: // 7
      returnvalue=7;
      break ;
      case 0xFF4AB5:  // 8
      returnvalue=8;
      break ;
      case 0xFF52AD: // 9
      returnvalue=9;
      break ;      
    }
    irrecv.resume();      
  }
  return returnvalue;
}

/*  ---------- Tidy ---------- */
/*Through this function, the Robotic Arm tidies  the pawns so they are in position for the next game */
void Tidy() {
  for(int  i=0;i<3;i++) {
    for(int j=0;j<3;j++) { 
      if(Tris[i][j]=='O') { pawn--;
        Reach_Position(TrisPos[i][j][0],TrisPos[i][j][1],TrisPos[i][j][2],TrisPos[i][j][3],TakePos[pawn][0],TakePos[pawn][1],TakePos[pawn][2]+10,TakePos[pawn][3]);  } 
    }
  }
}
