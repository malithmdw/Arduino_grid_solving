#include <Servo.h>

//button
#define buttonInPin A7
//motor pins
#define right_fw A2
#define right_bw A3
#define left_fw A4
#define left_bw A5

//tracking sensors     *2 *3 *4 ^5 *6 *7 *8
#define left_ex 3
#define left_2 2
#define left_1 4
#define sens_head 5
#define right_1 6

#define right_2 8
#define right_ex 7

//for arm
#define armcatchpin 30
#define armupdownpin 31
Servo ArmCatch;
Servo ArmUpDown;

//for distance sensor
#define trig 22
#define echo 23
long duration,distance;
//for distance sensor2
#define trig2 24
#define echo2 25
long duration2,distance2;
//for distance sensor3
#define trig3 26
#define echo3 27
long duration3,distance3;

//----------variables for our code---------
//for button
int buttonPressed=0;
//for tracking sensor inputs
int head;
int left1;
int left2;
int right1;
int right2;
int leftex;
int rightex;

//for path
int gridarray[7][10];
/*{
  {1,0,0,0,1,0,1,0,0,1},
  {1,0,0,0,1,1,1,0,0,1},
  {1,1,1,0,0,1,0,1,0,1},
  {1,0,1,1,0,1,0,0,0,1},
  {1,0,0,1,1,1,1,0,0,1},
  {0,0,0,0,0,0,1,1,1,1},
  {1,1,1,1,1,1,0,1,0,1},
  };*/

/* MAIN PATH FINDER RECURSION ALGORITHM */
int arr[7][10];
        /*{
           {1,1,1,1,1,1,0,1,0,1},
           {0,0,0,0,0,0,1,1,1,1},
           {1,0,0,1,1,1,1,0,0,1},
           {1,0,1,1,0,1,0,0,0,1},
           {1,1,1,0,0,1,0,1,0,1},
           {1,0,0,0,1,1,1,0,0,1},
           {1,0,0,0,1,0,1,0,0,1}
        };*/
int arr2[50];
int k=1;

int p=0;
int q=0;
int stopr=0;

/* MAIN PATH RUNNING   */

int curArrayPosition=0;
int nowgoing=4;

int lastarrpoint=1;//array position when return from main path visiting algorithm

int arrpositionX=6;//next index of node of grid array while traveling on edge
int arrpositionY=0;


/* sub path*/
int stops=0;
int r=0;
int s=0;

int ar2d1[7][10];
int arrsubpath1[50];

int repeats=0;
int stopforavoidrepeat=0;
/*  --------------------------   SETUP  ---------  START --------------------------------------------------------------------------------------------------------------------------------*/

void setup(){
  Serial.begin(9600);  
  //button
  pinMode(buttonInPin,INPUT);
    
  //motor pin outputs
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);

  //sensor pin inputs
  for(int i = 2; i < 7; i++){
    pinMode(i,INPUT);
  }
  //distance sensor pin input & output
  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
  //distance sensor2 pin input & output
  pinMode(trig2,OUTPUT);
  pinMode(echo2,INPUT);
  //distance sensor3 pin input & output
  pinMode(trig3,OUTPUT);
  pinMode(echo3,INPUT);
  
  //servo
  pinMode(armcatchpin, OUTPUT);
  pinMode(armupdownpin, OUTPUT); 
  ArmCatch.attach(armcatchpin);
  ArmCatch.write(0);
  ArmUpDown.attach(armupdownpin);
  ArmUpDown.write(0);

  /*run first round*/
  firstcoverage();
  
  /*calculate and finalize the directios of main path*/
  mainPathCalculator();
  Serial.println("end setup");
}










/*  --------------------------   MAIN LOOP  ---------  START --------------------------------------------------------------------------------------------------------------------------------*/
void loop(){
  if(digitalRead(buttonInPin)== HIGH){
    if(buttonPressed==0){
      buttonPressed=1;
      mainpathrunning();

      buttonPressed=0;
      
      //variable refresh for new round
      for (int a=0;a<7;a++){
        for (int b=0;b<10;b++){
          ar2d1[a][b]=gridarray[a][b];
        }  
      }
      for(int i=1;i<50;i++){
        arrsubpath1[i]=0;
        arr2[i]=0;
      }
      k=1;
      p=0;
      q=0;
      stopr=0;
      curArrayPosition=0;
      nowgoing=4;
      lastarrpoint=1;
      arrpositionX=6;
      arrpositionY=0;
      /* sub path*/
      stops=0;
      r=0;
      s=0;
      
      repeats=0;
      stopforavoidrepeat=0;
    }
  }    
}
/*  --------------------------   MAIN LOOP  -------   END   ---------------------------------------------------------------------------------------------------------------------------------*/

















/* ----------------------------COMMON  FUNCTIONS   -------------------------------*/
void printGridAndPathArray(){
    Serial.println("GRID Printing START");
    for(int i=0;i<7;i++){//for 7 rows
      for(int j=0;j<10;j++){//for 10 colums
        Serial.print(gridarray[i][j]);
      }
        Serial.println("");
    }
    Serial.println("GRID Printing END");
    
    for(int l=0;l<25;l++){
      Serial.print(arr2[l]);
    }
    Serial.println("");  
}

void updatesensors(){ //update inputs of tracking sensors
  head=digitalRead(sens_head);
  left1=digitalRead(left_1);
  left2=digitalRead(left_2);
  right1=digitalRead(right_1);
  right2=digitalRead(right_2);
  leftex=digitalRead(left_2);//(leftex);
  rightex=digitalRead(right_2);//(rightex);
}

void goFWD(){
  digitalWrite(right_fw,1);
  digitalWrite(left_fw,1);
}
void goBWD(){
  digitalWrite(right_bw,1);
  digitalWrite(left_bw,1);
}
void stopWHL(){
  digitalWrite(right_fw,0);
  digitalWrite(left_fw,0);
  digitalWrite(right_bw,0);
  digitalWrite(left_bw,0);
}
void leftcorrection(){//outing black line from left side
  stopWHL();
  updatesensors();
  while(1){
    updatesensors();
    digitalWrite(left_fw,1);
    if(left1==1 && right1==1){
      stopWHL();
      break;
    }
    if(right2==0 || left2==0){
      stopWHL();
      break;
    }
  }
  stopWHL();
}
void rightcorrection(){//outing black line from right side
  stopWHL();
  updatesensors();
  while(1){
    updatesensors();
    digitalWrite(right_fw,1);
    if(left1==1 && right1==1){
      stopWHL();
      break;
    }
    if(right2==0 || left2==0){
      stopWHL();
      break;
    }
  }
  stopWHL();
}
void turnLFT90(){
  Serial.println("turn <");
  stopWHL();
  digitalWrite(right_fw,1);
  delay(400);
  stopWHL();
  updatesensors();
  while(head==1){
    updatesensors();
    digitalWrite(right_fw,1);
    digitalWrite(left_bw,1);
    if(head==0){
      stopWHL();
      break;
    }
    if(left1==0 && right1==0 && left2==0 && right2==0 && head==0){
      stopWHL();
      break;
    }
  }
  stopWHL();
}
void turnRGHT90(){
  Serial.println("turn >");
  stopWHL();
  digitalWrite(left_fw,1);
  delay(400);
  stopWHL();
  updatesensors();
  while(head==1){
    updatesensors();
    digitalWrite(left_fw,1);
    digitalWrite(right_bw,1);
    if(head==0){
      stopWHL();
      break;
    }
    if(left1==0 && right1==0 && left2==0 && right2==0 && head==0){
      stopWHL();
      break;
    }
  }
  stopWHL();
}
void turn180(){ 
  updatesensors();
    while(right2==0){
      updatesensors();
      digitalWrite(left_fw,1);
      digitalWrite(right_bw,1);
      if(right2==1){
        stopWHL();
        break;
      }
    }
    stopWHL();
    while(right2==1){
      updatesensors();
      digitalWrite(left_fw,1);
      digitalWrite(right_bw,1);
      if(right2==0){
        stopWHL();
        break;
      }
    }
    while(right2==0){
      updatesensors();
      digitalWrite(left_fw,1);
      digitalWrite(right_bw,1);
      if(right2==1){
        stopWHL();
        break;
      }
    }
    stopWHL();
    while(right2==1){
      updatesensors();
      digitalWrite(left_fw,1);
      digitalWrite(right_bw,1);
      if(right2==0 || right2==0){
        stopWHL();
        break;
      }
    }
  
  while(head==1){
    updatesensors();
    digitalWrite(left_fw,1);
    digitalWrite(right_bw,1);
    if(head==0 && (left1==1 || right1==1)){
      stopWHL();
      break;
    }
  }
}
int goFWwithCorrections(){
  while(1){
    updatesensors();
    if(left2 == 0 || right2 == 0 || leftex==0 || rightex==0){
      stopWHL();
      break;     
    }
    if(left1==0 && right1==0){
      break;
    }
    if(left1==0){
      rightcorrection();
      stopWHL();
    }
    if(right1==0){
      leftcorrection();
      stopWHL();
    }
    goFWD();
  }
  stopWHL();
  return 0;
}
int goFWwithCorrectionsANDcheckPits(int distancetostop,int option){
  long dst;
  while(1){
    updatesensors();
    dst=distanceSensor3();
    if(option==1){//check the pit with 10*10*10
      if(dst>distancetostop){
        int pass=1;
        for(int i=0;i<25;i++){//to get more correct decision
          dst=distanceSensor();
          if(dst<distancetostop){
            pass=0;
          }
        }
        if(pass==1){
          stopWHL();
          break;
        }
      }
      if(left2 == 0 || right2 == 0 || leftex==0 || rightex==0){
        stopWHL();
        break;     
      }
      if(left1==0 && right1==0){
        break;
      }
    }else if(option==2){// check other pits
      if(dst<(distancetostop+0.3) && dst>(distancetostop-0.3)){
        int pass=1;
        for(int i=0;i<25;i++){//to get more correct decision
          dst=distanceSensor();
          if(!(dst<(distancetostop+0.3) && dst>(distancetostop-0.3))){
            pass=0;
          }
        }
        if(pass==1 && left1==0 && right1==0){
          stopWHL();
          break;
        }
      }           
    }  
    if(left1==0){
      rightcorrection();
      stopWHL();
    }
    if(right1==0){
      leftcorrection();
      stopWHL();
    }  
    goFWD();
  }
  stopWHL();
  return 0;
}
int goFWwithCorrectionsANDdistance(int distancetostop){
  long dst;
  while(1){
    updatesensors();
    dst=distanceSensor();
    if(dst<=distancetostop){
      int pass=1;
      for(int i=0;i<25;i++){//to get more correct decision
        dst=distanceSensor();
        if(dst>distancetostop){
          pass=0;
        }
      }
      if(pass==1){
        stopWHL();
        break;
      }
    }
    if(left2 == 0 || right2 == 0 || leftex==0 || rightex==0){
      stopWHL();
      break;     
    }
    if(left1==0 && right1==0){
      break;
    }
    if(left1==0){
      rightcorrection();
      stopWHL();
    }
    if(right1==0){
      leftcorrection();
      stopWHL();
    }
    goFWD();
  }
  stopWHL();
  return 0;
}
int goFWwithCorrections4cm(){
  while(1){
    updatesensors();
    if(left2 == 0 || right2 == 0 || leftex==0 || rightex==0){
      stopWHL();
      break;     
    }
    if(left1==1){
      stopWHL();
      updatesensors();
      while(1){
        updatesensors();
        digitalWrite(left_fw,1);
        if(left1==0 && right1==0){
          stopWHL();
          break;
        }
        if(rightex==0 || leftex==0){
          stopWHL();
          break;
        }
      }
      stopWHL();
    }
    if(right1==1){
      stopWHL();
      updatesensors();
      while(1){
        updatesensors();
        digitalWrite(right_fw,1);
        if(left1==0 && right1==0){
          stopWHL();
          break;
        }
        if(rightex==0 || leftex==0){
          stopWHL();
          break;
        }
      }
      stopWHL();
    }
    goFWD();
  }
  stopWHL();
  return 0;
}
int goBWwithCorrections4cm(){
  while(1){
    updatesensors();
    if(left2 == 0 || right2 == 0 || leftex==0 || rightex==0){
      stopWHL();
      break;     
    }
    if(left1==1){
      stopWHL();
      updatesensors();
      while(1){
        updatesensors();
        digitalWrite(right_bw,1);
        if(left1==0 && right1==0){
          stopWHL();
          break;
        }
        if(right2==0 || left2==0){
          stopWHL();
          break;
        }
      }
      stopWHL();
    }
    if(right1==1){
      stopWHL();
      updatesensors();
      while(1){
        updatesensors();
        digitalWrite(left_bw,1);
        if(left1==0 && right1==0){
          stopWHL();
          break;
        }
        if(right2==0 || left2==0){
          stopWHL();
          break;
        }
      }
      stopWHL();
    }
    goBWD();
  }
  stopWHL();
  return 0;
}
int goBWwithCorrections(){
  while(1){
    updatesensors();
    if(left2 == 0 || right2 == 0 || leftex==0 || rightex==0){
      stopWHL();
      break;     
    }
    if(left1==0 && right1==0){
      break;
    }
    if(left1==0){
       stopWHL();
       while(1){
        updatesensors();
        digitalWrite(right_bw,1);
        if(left1==1 && right1==1){
          stopWHL();
          break;
        }
        if(right2==0 || left2==0){
          stopWHL();
          break;
        }
      }
      stopWHL();
    }
    if(right1==0){
      stopWHL();
      while(1){
        updatesensors();
        digitalWrite(left_bw,1);
        if(left1==1 && right1==1){
          stopWHL();
          break;
        }
        if(right2==0 || left2==0){
          stopWHL();
          break;
        }
      }
      stopWHL();
    }
    goBWD();
  }
  stopWHL();
  return 0;
}
void passnode(){
  stopWHL();
  while(1){
    updatesensors();
    if(left2 == 1 && right2 == 1 && (left1 == 1 || right1 == 1)){
      stopWHL();
      break;                              
    }
    goFWD();
  }
  stopWHL();
}



long distanceSensor(){//find object
  digitalWrite(trig,LOW);
  delayMicroseconds(2);

  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);

  duration=pulseIn(echo,HIGH);
  distance=duration/58;
  return (distance);
}
long distanceSensor2(){//mesure the payload   not completed,copy of before
  digitalWrite(trig2,LOW);
  delayMicroseconds(2);

  digitalWrite(trig2,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2,LOW);

  duration2=pulseIn(echo2,HIGH);
  distance2=duration2/58;
  return (distance2);
}
long distanceSensor3(){//mesure pits
  digitalWrite(trig3,LOW);
  delayMicroseconds(2);

  digitalWrite(trig3,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig3,LOW);

  duration3=pulseIn(echo3,HIGH);
  distance3=duration3/58;
  return (distance2);
}

/* END ----------------------------------------------------------------------COMMON  FUNCTIONS   -----------------------------------------------------------------------------------------*/


















/*grid calibration*/
void firstcoverage(){
  
  int x=6;
  int y=9;
  int curPassNodeInRow=1;
  //go foward
  while(1){
    if(left2==1 && right2==1){
      stopWHL();
      break;
    }
    goFWD();
  }
  goFWwithCorrections();
  //turn right
  turnRGHT90();
  //grid

    int curPassRow=1;
    for(int i=0;i<7;i++){//for 7 rows
      Serial.print("Row >");Serial.println(i+1);
      curPassRow=i+1; 
      for(int j=1;j<10;j++){//for 10 colums
        Serial.println(j);
        updatesensors();
        goFWwithCorrections();

                
        //check first row?
        if(curPassRow==1){
          if(j==9){//turn ]
            updatesensors();
            if(head==1){// _|_ blank
              gridarray[x-i][j]=0;
            }
            if(head==0){// _|_ filled
              gridarray[x-i][j]=1;
            }           
            turnLFT90();
            stopWHL();
            goFWwithCorrections();
            stopWHL();
            if(left2==0 || leftex==0){
              if(head==1){// _|_ blank
                gridarray[x-i-1][j]=0;
              }
              if(head==0){// _|_ filled
                gridarray[x-i-1][j]=1;
              }              
            }
            turnLFT90();
            stopWHL();
          }else{
            updatesensors();
            if(head==0){// _|_ filled
               gridarray[x-i][j]=1;              
             }else{// _|_ blank
               gridarray[x-i][j]=0;
             }
             while(1){
              updatesensors();
              if(left2 == 1 && right2 == 1 && leftex == 1){
                stopWHL();
                break;                              
              }
              goFWD();
             }
             stopWHL();
          }
        

        
        //chech last row?
        }else if(curPassRow==7){
          if(j==9){//stop, grid is covered
            stopWHL();
            updatesensors();
            if(head==1){// T filled
              gridarray[x-i][j]=0;
            }  
            if(head==0){// T blank
              gridarray[x-i][j]=1;
            }          
          }else{
            updatesensors();
            if(head==1){// T filled
                gridarray[x-i][j]=0;
            }
            if(head==0){// T blank
              gridarray[x-i][j]=1;
            }
            while(1){
              updatesensors();
              if(left2 == 1 && right2 == 1 && rightex == 1){
                stopWHL();
                break;                              
              }
              goFWD();
            }
            stopWHL();  
          }
        }else{
          if((j+1)==10){//end of a line  //check want turn bend?
            if((i%2)==0){// dicission > bend for LEFT
                stopWHL();
                updatesensors();
                if(head==0){
                  gridarray[x-i][j]=1;
                }else if(head==1){
                  gridarray[x-i][j]=0;
                }
                  turnLFT90();
                  stopWHL();
                  goFWwithCorrections();
                  stopWHL();
                  updatesensors();
                  if(head==0){
                    gridarray[x-i-1][y]=1;
                  }else{
                    gridarray[x-i-1][y]=0;
                  }
                  turnLFT90(); 
                  stopWHL(); 
            }else if((i%2)==1){// decission > bend for RIGHT 
                stopWHL();
                updatesensors();
                if(head==0){
                  gridarray[x-i][0]=1;
                }else if(head==1){
                  gridarray[x-i][0]=0;
                }
                  turnRGHT90();
                  stopWHL();
                  goFWwithCorrections();
                  stopWHL();
                  updatesensors();
                  if(head==0){
                    gridarray[x-i-1][0]=1;
                  }else{
                    gridarray[x-i-1][0]=0;
                  }
                  turnRGHT90();
                  stopWHL();  
            }
            
          //normal node
          
          }else{
            if((i%2)==0){//going > side(j 's increment is ok)
              updatesensors();
              if(head==0){// T blank
                gridarray[x-i][j]=1;            
              }else if(head==1){// T filled
                gridarray[x-i][j]=0;
              }
              while(1){
                  updatesensors();
                  if(left2 == 1 && right2 == 1 && (left1 == 1 || right1 == 1)){
                    stopWHL();
                    break;                              
                  }
                  goFWD();
                }
                stopWHL();
            }else if((i%2)==1){//going < side(j 's increment should be inverse)
              updatesensors();
              if(head==0){// T blank
                gridarray[x-i][y-j]=1;            
              }else if(head==1){// T filled
                gridarray[x-i][y-j]=0;
              }
                while(1){
                  updatesensors();
                  if(left2 == 1 && right2 == 1 && (left1 == 1 || right1 == 1)){
                    stopWHL();
                    break;                              
                  }
                  goFWD();
                }
                stopWHL();
            }
          }
        }
        curPassNodeInRow=curPassNodeInRow+1;
        } 
        curPassNodeInRow=1;      
      }

      gridarray[6][0]=1;
      gridarray[6][9]=1;
      gridarray[6][4]=1;
      gridarray[6][6]=1;
      if(gridarray[6][8]==0 && gridarray[5][9]==0){
        gridarray[6][8]=1;
        gridarray[5][9]=1;
      }
      if(gridarray[6][0]==0 && gridarray[5][0]==0){
        gridarray[6][0]=1;
        gridarray[5][0]=1;
      }
}






void mainPathCalculator(){
  stopr=0;
  for (int s=0;s<7;s++){
    for (int t=0;t<10;t++){
      arr[s][t]=gridarray[s][t];
    }  
  }
  mypath(6,0);
}
void subpathcalculator(int r,int s,int t,int u){
  stops=0;
  k=1;
  stopforavoidrepeat=0;
  repeats=0;
  for (int a=0;a<7;a++){
    for (int b=0;b<10;b++){
      ar2d1[a][b]=gridarray[a][b];
    }  
  }
  for(int i=1;i<50;i++){
    arrsubpath1[i]=0;
  }  
  mysubpathR(r,s,t,u);
}


/*main path calculaion recursive functin*/
int mypath(int p,int q){
    if(p==6 && q==9){
        stopr=1;
        Serial.println("discovered");
      return 0;  
    }
    int rightexist=1;
    int downexist=1;
    int upexist=1;
    int backexist=1;
    if(q == 9){rightexist=0;}
    if(p == 6){downexist=0;}
    if(p == 0){upexist=0;}
    if(q == 0){backexist=0;}
    
    if(stopr==1){
        return 0;
    }else{
        if(rightexist==1 && arr2[k-1] != 3 && stopr==0){
            if(arr[p][q+1]==1){
               if(arr2[k-1]==4 && arr2[k-2]==3 && arr2[k-3]==2 && arr2[k-4]==1){// a cycle
                    arr[p+1][q+1]=0;
                    k=k-4;
                }else{
                    arr2[k]=1;
                    k=k+1;
                    mypath(p,q+1); 
                }           
            }  
        }
        if(downexist==1 && arr2[k-1] != 4 && stopr==0){
            if(arr[p+1][q]==1){
              if(arr2[k-1]==1 && arr2[k-2]==4 && arr2[k-3]==3 && arr2[k-4]==2){// a cycle
                arr[p+1][q-1]=0;                    
                k=k-4;
              }else{
                arr2[k]=2;
                k=k+1;
                mypath(p+1,q);  
              }         
           }  
        }
        if(upexist==1 && arr2[k-1] != 2 && stopr==0){
            if(arr[p-1][q]==1){
              if(arr2[k-1]==1 && arr2[k-2]==2 && arr2[k-3]==3 && arr2[k-4]==4){// a cycle
                arr[p-1][q-1]=0;                    
                k=k-4;
              }else{
                 arr2[k]=4;
                 k=k+1;
                 mypath(p-1,q);
              }           
            }  
        }
        if(backexist==1 && arr2[k-1] != 1 && stopr==0){
            if(arr[p][q-1]==1){
               arr2[k]=3;
               k=k+1;
               mypath(p,q-1);           
            }  
        }
        if(stopr==0){
            arr[p][q]=0;
            k=k-1;
            switch(arr2[k]){
                case 1:mypath(p,q-1);//>
                case 2:mypath(p-1,q);//
                case 3:mypath(p,q+1);//<
                case 4:mypath(p+1,q);//^
            }
        }
    }    
    return 0;    
}
/*sub path calculaion recursive functin*/
int mysubpathR(int r,int s,int t,int u){
    if(r==t && s==u){
      stops=1;
      return 0;  
    }
    int rightexist=1;
    int downexist=1;
    int upexist=1;
    int backexist=1;
    if(s == 9){rightexist=0;}
    if(r == 6){downexist=0;}
    if(r == 0){upexist=0;}
    if(s == 0){backexist=0;}
    
    if(stops==1){
        return 0;
    }else{
        if(downexist==1 && arrsubpath1[k-1] != 4 && stops==0){            
            if(ar2d1[r+1][s]==1){
                if(arrsubpath1[k-1]==1 && arrsubpath1[k-2]==4 && arrsubpath1[k-3]==3 && arrsubpath1[k-4]==2){// a cycle
                    ar2d1[r+1][s-1]=0;                    
                    k=k-4;
                }else{
                    arrsubpath1[k]=2;
                    k=k+1;
                    mysubpathR(r+1,s,t,u); 
                }
            }  
        }
        if(rightexist==1 && arrsubpath1[k-1] != 3 && stops==0){
            if(ar2d1[r][s+1]==1){
                if(arrsubpath1[k-1]==4 && arrsubpath1[k-2]==3 && arrsubpath1[k-3]==2 && arrsubpath1[k-4]==1){// a cycle
                    ar2d1[r+1][s+1]=0;                    
                    k=k-4;
                }else{
                    arrsubpath1[k]=1;
                    k=k+1;
                    mysubpathR(r,s+1,t,u); 
                }                               
            }  
        }
        if(backexist==1 && arrsubpath1[k-1] != 1 && stops==0){
            if(ar2d1[r][s-1]==1){
                if(arrsubpath1[k-1]==4 && arrsubpath1[k-2]==1 && arrsubpath1[k-3]==2 && arrsubpath1[k-4]==3){// a cycle
                    ar2d1[r-1][s-1]=0;                    
                    k=k-4;
                }else{
                    arrsubpath1[k]=3;
                    k=k+1;
                    mysubpathR(r,s-1,t,u);
                }
            }  
        }        
        if(upexist==1 && arrsubpath1[k-1] != 2 && stops==0){
            if(ar2d1[r-1][s]==1){
                arrsubpath1[k]=4;
                k=k+1;
                mysubpathR(r-1,s,t,u);
               
            }  
        }
        
        if(stops==0){
            ar2d1[r][s]=0;
            k=k-1;
            switch(arrsubpath1[k]){
                case 1:mysubpathR(r,s-1,t,u);//>
                case 2:mysubpathR(r-1,s,t,u);//v
                case 3:mysubpathR(r,s+1,t,u);//<
                case 4:mysubpathR(r+1,s,t,u);//^
            }
        }
    }    
    return 0;
}

















/*main path visiting procedure functin*/
void mainpathvisitalgorithm(int startpointfromarray){
  long dis;
  for(int i=startpointfromarray;i<50;i++){
    curArrayPosition=i;
    dis=distanceSensor();
    lastarrpoint=i;
    
    if(arr2[i] == 0){
      break;
    }
    if(dis<=10){
      break;
    }
    if(nowgoing==1){//going > side
      //Serial.println("       going >");
      int x=arr2[i];
      switch(x){
        case 1:passnode();arrpositionY=arrpositionY+1;break;//pass the node 
        case 2:turnRGHT90();arrpositionX=arrpositionX+1;break;
        case 3:turn180();arrpositionY=arrpositionY-1;break;
        case 4:turnLFT90();arrpositionX=arrpositionX-1;break;
      }
      nowgoing=arr2[i];     
    }else if(nowgoing==2){////going v side
      //Serial.println("       going v");
      switch(arr2[i]){
        case 1:turnLFT90();arrpositionY=arrpositionY+1;break;
        case 2:passnode();arrpositionX=arrpositionX+1;break;//pass the node
        case 3:turnRGHT90();arrpositionY=arrpositionY-1;break;
        case 4:turn180();arrpositionX=arrpositionX-1;break;
      }
      nowgoing=arr2[i];
    }else if(nowgoing==3){//going < side
      //Serial.println("       going <");
      switch(arr2[i]){
        case 1:turn180();arrpositionY=arrpositionY+1;break; 
        case 2:turnLFT90();arrpositionX=arrpositionX+1;break;
        case 3:passnode();arrpositionY=arrpositionY-1;break;//pass the node
        case 4:turnRGHT90();arrpositionX=arrpositionX-1;break;
      }
      nowgoing=arr2[i];
    }else if(nowgoing==4){//going ^ side
      //Serial.println("       going ^");
      int x=arr2[i];
      switch(x){
        case 1:turnRGHT90();arrpositionY=arrpositionY+1;break; 
        case 2:turn180();arrpositionX=arrpositionX+1;break;
        case 3:turnLFT90();arrpositionY=arrpositionY-1;break;
        case 4:passnode();arrpositionX=arrpositionX-1;break;//pass the node
      }
      nowgoing=arr2[i]; 
    }
    goFWwithCorrectionsANDdistance(10);
  }
}

/*sub path visiting procerure functin*/
void subpathvisitalgorithm(){
  int arrelemntcount=0;
  for(int i=1;i<50;i++){
    if(arrsubpath1[i]==0){
      break;
    }
    arrelemntcount=arrelemntcount+1;
  }

  
  for(int i=1;i<50;i++){
    if(arrsubpath1[i] == 0){
      break;
    }
    if((arrelemntcount-stopforavoidrepeat+1)==i){ 
      curArrayPosition=curArrayPosition+stopforavoidrepeat;
      stopforavoidrepeat=0;
      break;
    }
    if(nowgoing==1){//going > side
      //Serial.println("       going >");
      switch(arrsubpath1[i]){
        case 1:passnode();break;//pass the node 
        case 2:turnRGHT90();break;
        case 3:turn180();break;
        case 4:turnLFT90();break;
      }
      nowgoing=arrsubpath1[i];     
    }else if(nowgoing==2){////going v side
      //Serial.println("       going v");
      switch(arrsubpath1[i]){
        case 1:turnLFT90();break;
        case 2:passnode();break;//pass the node
        case 3:turnRGHT90();break;
        case 4:turn180();break;
      }
      nowgoing=arrsubpath1[i];
    }else if(nowgoing==3){//going < side
      //Serial.println("       going <");
      switch(arrsubpath1[i]){
        case 1:turn180();break; 
        case 2:turnLFT90();break;
        case 3:passnode();break;//pass the node
        case 4:turnRGHT90();break;
      }
      nowgoing=arrsubpath1[i];
    }else if(nowgoing==4){//going ^ side
      //Serial.println("       going ^");
      switch(arrsubpath1[i]){
        case 1:turnRGHT90();break; 
        case 2:turn180();break;
        case 3:turnLFT90();break;
        case 4:passnode();//pass the node
      }
      nowgoing=arrsubpath1[i]; 
    }
     goFWwithCorrections();
  }
}
/*reversing array and make a path to return fron payload bay*/
void returnfromsubpath(){
  int count=0;
  stopforavoidrepeat=0;
  for(int i=1;i<50;i++){//count repeats for avoid repeats
    if(arrsubpath1[i]== arr2[i+curArrayPosition-1]){
      stopforavoidrepeat=stopforavoidrepeat+1;
      repeats=repeats+1;
    }else{
      break;
    }
  }
    
  for(int i=1;i<50;i++){
    if(arrsubpath1[i]==0){
        break;
    }
    switch(arrsubpath1[i]){
        case 1:arrsubpath1[i]=3;break;
        case 2:arrsubpath1[i]=4;break;
        case 3:arrsubpath1[i]=1;break;
        case 4:arrsubpath1[i]=2;break;
    }
    count=count+1;
  }
  if(count%2==0){
    for(int i=1;i<(count/2)+1;i++){
        int temp=arrsubpath1[i];
        arrsubpath1[i]=arrsubpath1[count-i+1];
        arrsubpath1[count-i+1]=temp;
    }
  }else{
     for(int i=1;i<((count-1)/2)+1;i++){
        int temp=arrsubpath1[i];
        arrsubpath1[i]=arrsubpath1[count-i+1];
        arrsubpath1[count-i+1]=temp;
    } 
  }  
}


/*highlevel functions*/
void goTo1010(){
  subpathcalculator(arrpositionX,arrpositionY,6,4);
  subpathvisitalgorithm();
  //final turn
  switch(nowgoing){
    case 1:turnRGHT90();break; 
    case 2:passnode();break;
    case 3:turnLFT90();break;
    case 4:turn180();break;
  }
  nowgoing=2;
}
void goTo88(){
  subpathcalculator(arrpositionX,arrpositionY,6,6);
  subpathvisitalgorithm();
  //final turn
  switch(nowgoing){
    case 1:turnRGHT90();break; 
    case 2:passnode();break;
    case 3:turnLFT90();break;
    case 4:turn180();break;
  }
  nowgoing=2;
}

void backFromPayloadBay88or1010(){
  returnfromsubpath();
  subpathvisitalgorithm();    
  for(int i=0;i<repeats;i++){
    switch(arr2[lastarrpoint+i]){
      case 1:arrpositionY=arrpositionY+1;break;
      case 2:arrpositionX=arrpositionX+1;break;
      case 3:arrpositionY=arrpositionY-1;break;
      case 4:arrpositionX=arrpositionX-1;break;      
    }
  }
  repeats=0;
}
















/*overrol function to complete the task*/
void mainpathrunning(){
  
  //start
  /*go foward while passed the black ariena*/
  while(1){
    updatesensors();
    if(left2==1 && right2==1){
      stopWHL(); 
      break;
    }
    goFWD();
  }
  stopWHL(); 
  goFWwithCorrections();
    
  /*enter to grid  */
  curArrayPosition=0;
  nowgoing=4;
  arrpositionX=6;
  arrpositionY=0;
  
  mainpathvisitalgorithm(1);
  stopWHL();
  
  //MEET the 1st and 2nd  PAYLOADs............................................................... 
  for(int r=0;r<2;r++){
  
  /*arrange arm widely*/
  ArmCatch.write(0);
  goFWwithCorrectionsANDdistance(3);
  /*load the payload*/
  ArmCatch.write(90);
  ArmUpDown.write(90);
  /*messure the payload size*/
  int widthis10=0;
  int widthis8=0;
  long ploadsize;
    
  for(int i=0;i<25;i++){
    ploadsize=distanceSensor2();
    if(ploadsize<=9){
      widthis8=widthis8+1;
    }else{
      widthis10=widthis10+1;
    }
  }
  goFWwithCorrections();// go while meet the junctin to get decission
  stopWHL();

  
  /* go to correct bay and unload and Return to last grid position */
  if(widthis10>=widthis8){//go to pay load bay 10*10
    goTo1010();
    stopWHL();
    passnode();
    stopWHL();
    goFWwithCorrections4cm();
    stopWHL();
    goBWD();
    delay(200);
    stopWHL();
    
    /*unload the payload*/
    ArmUpDown.write(0);
    ArmCatch.write(0);
  
    goBWwithCorrections4cm();
    stopWHL();
    backFromPayloadBay88or1010();
    stopWHL(); 
    
  }else{//go to pay load bay 8*8
    goTo88();
    stopWHL();
    passnode();
    stopWHL();
    goFWwithCorrections();
    stopWHL();
    goBWD();
    delay(200);
    stopWHL();
    /*unload the payload*/
    ArmUpDown.write(0);
    ArmCatch.write(0);
    
    goBWwithCorrections();
    stopWHL();
    backFromPayloadBay88or1010();//come back
    stopWHL();
  }
  /* again start grid covering from last break point */
  mainpathvisitalgorithm(curArrayPosition);
  stopWHL();

  }
  
  stopWHL();
  /* Again travel for meet 3rd payload*/
   //MEET the 3rd  PAYLOAD............................................................... 
  
  /*arrange arm widely*/
  
  goFWwithCorrectionsANDdistance(3);
  /*load the payload*/
  ArmCatch.write(90);
  ArmUpDown.write(90); 
  
  goFWwithCorrections(); 
  mainpathvisitalgorithm(curArrayPosition);
  
  



  
  /* NOW we are at the end point of grid*/
  //FINAL PART........................................................................
    
  switch(nowgoing){//pass the last node of grid
    case 1:passnode();break;
    case 2:turnLFT90();break;    
  }
  stopWHL();
  
  goFWwithCorrectionsANDcheckPits(7,1);//int distancetostop,int option
  stopWHL();
  goBWD();
  delay(200);
  stopWHL();
  //relese arm for unload & unload
  ArmUpDown.write(0);
  ArmCatch.write(0);  
  goFWwithCorrections();

  
  turnLFT90();//bend
  goFWwithCorrectionsANDcheckPits(7,2);
  
  for(int i=0;i<3;i++){
    while(1){
      goFWwithCorrections();
      stopWHL();
      goFWD();
      delay(300);
      stopWHL();
      if(left1==0 && right1==0 && head==0){
        stopWHL();
        break;
      }
    }    
  }  
  //stop  
}

