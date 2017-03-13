

#include <EEPROM.h>

/*
 * 
 * LINE FOLLOWER ROBOT WITH P-TYPE FEEDBACK
 * WORKS FINE FOR TURNS <= 90 DEGREE
 * 
 * 
 */

boolean implPhase = true;
int addr = 0;

//Right motor
int M1_PIN1 = 6;
int M1_PIN2 = 7;

//Left motor
int M2_PIN1 = 4;
int M2_PIN2 = 5;

//Sensors(Analog & Digital)
int sensors[5]={0,0,0,0,0};
int input[5]={0,0,0,0,0};
int last_avg=0;


//MAZE DATA
int pathIndex=0;
int const MAX_ARRAY_SIZE = 100;
char path[MAX_ARRAY_SIZE];
int LENGTH = 0;
/*
 *    PID Constants   
 *    
 * Kp = Max_response/Max_error_Value
 * Hence, Kp = 10/4 = 2.5
 */
float Kp = 2.5;


void readPath(){
  LENGTH = EEPROM.read(addr++);
  if(LENGTH > MAX_ARRAY_SIZE)
    return;
  for(int i=0;i<LENGTH;i++){
    path[i] = EEPROM.read(addr++);
    if(addr == EEPROM.length())
      addr = 0;
  }
}

/*
 *  To turn robot towards left
 *  Right motor is On
 *  pwmOnTime -> value between (0 and 10)
 */
void rightMotorOn(int pwmOnTime){
  if(pwmOnTime > 10){
    pwmOnTime = 10;
  }
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, LOW);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, HIGH);
    delay(pwmOnTime);
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, HIGH);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, HIGH);
    delay(10 - pwmOnTime);
  }

  

 /*
 *  To turn robot towards right
 *  Left motor is On
 *  pwmOnTime -> value between (0 and 10)
 */
void leftMotorOn(int pwmOnTime){
  if(pwmOnTime > 10){
    pwmOnTime = 10;
  }
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, HIGH);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, LOW);
    delay(pwmOnTime);
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, HIGH);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, HIGH);
    delay(10 - pwmOnTime);
  }


void rotateCloclwiseOnAxis(int angle){
    long t = 6.25 * angle;
    digitalWrite(M1_PIN1, LOW);
    digitalWrite(M1_PIN2, HIGH);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, LOW);
    delay(t);
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, HIGH);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, HIGH);
  }

  void rotateAntiCloclwiseOnAxis(int angle){
    long t = 6.25 * angle;
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, LOW);
    digitalWrite(M2_PIN1, LOW);
    digitalWrite(M2_PIN2, HIGH);
    delay(t);
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, HIGH);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, HIGH);
  }

  
/*
 * m1 and m2 max value = 10;
 * m2 - left motor
 * m1 - right motor
 */
void motorSpeed(int m2,int m1){

    int totalOnTime = 0;
    if(m1 > 10 || m2 > 10 || m1 < 0 || m2 < 0)
        return;
    if(m1 == 0 && m2 == 0){
        stopAllMotors();
        return;
    }
    if(m1 == m2){
        bothMotorOnForward(m1);
        return;
    }
    if(m1 == 0){
        leftMotorOn(m2);
        return;
    }
    if(m2 == 0){
        rightMotorOn(m1);
        return;
    }
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, LOW);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, LOW);
    if(m2 > m1){
        totalOnTime = m2;
      // Stop motor m1 after a delay of 'm1' mS
        delay(m1);
        digitalWrite(M1_PIN1, HIGH);
        digitalWrite(M1_PIN2, HIGH);
        // Stop both the motors after a delay of 'm2' mS
        delay(m2-m1);
    }else{
      
        totalOnTime = m1;
      // Stop motor m2 after a delay of 'm2' mS
        delay(m2);
        digitalWrite(M2_PIN1, HIGH);
        digitalWrite(M2_PIN2, HIGH);
        // Stop both the motors after a delay of 'm2' mS
        delay(m1-m2);
      }
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, HIGH);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, HIGH);
    delay(10 - totalOnTime);
  }


 /*
 *  Turn On both motors
 *  Both Motor is On
 *  pwmOnTime -> value between (0 and 10)
 */
void bothMotorOnForward(int pwmOnTime){
  if(pwmOnTime > 10){
    pwmOnTime = 10;
  }
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, LOW);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, LOW);
    delay(pwmOnTime);
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, HIGH);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, HIGH);
    delay(10 - pwmOnTime);
  }

 /*
 *  Turn On both motors in  backward direction
 *  Both Motor is On in backward direction
 *  pwmOnTime -> value between (0 and 10)
 */
void bothMotorOnReverse(int pwmOnTime){
  if(pwmOnTime > 10){
    pwmOnTime = 10;
  }
    digitalWrite(M1_PIN1, LOW);
    digitalWrite(M1_PIN2, HIGH);
    digitalWrite(M2_PIN1, LOW);
    digitalWrite(M2_PIN2, HIGH);
    delay(pwmOnTime);
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, HIGH);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, HIGH);
    delay(10 - pwmOnTime);
  }



void rotationController(int rotationDirection){
      moveForward(400);
      stopAllMotors();
      int angle = 0;
      do{
        if(rotationDirection > 0)
            rotateCloclwiseOnAxis(10);
        else
            rotateAntiCloclwiseOnAxis(10);
      angle +=  10;
      readSensors();
      analogToDigital();
      if(input[2] == 1)
        break;
      }while(angle<180);
}


/*
 *      PID Controller
 *      
 * response should be max 10 - max pwm possible
 * but to slowdown the entire rbot we r keeping 
 * max response to be 4, i.e. Max_response = 4
 * 
 * thus, Kp = Max_response / Max_error;
 * Here, Max_error = 4 thus Kp = 1
 */

void pidController(int error){
    float response = Kp * error;
    int M1_speed = Kp*4 - response;
    int M2_speed = Kp*4 + response;
    if(M1_speed > 10)
        M1_speed = 10;
    if(M2_speed > 10)
        M2_speed = 10;
        
    if(M1_speed < 0)// abstract case : this will only happen if there is a error
        M1_speed = 0;
    if(M2_speed < 10)// abstract case : this will only happen if there is a error
        M2_speed = 0;
    motorSpeed( M2_speed  ,M1_speed ); 
    //Serial.println("Straight " );

}




void motorController(int error){
      /*
       * ERROR=1,2,3,4 -> PID CONTROLLER
       * ERROR=5,6,7,8 -> ROTATE LEFT/RIGHT  
       * ERROR=56      -> ROBOT ON BLACK SURFACE,HENCE STOP
       * ERROR=64      -> ROBOT ON WHITE SURFACE,HENCE MOVE STRAIGHT
       */
    if(error >= -4 && error <= 4){
        pidController(error);
        return;
    }
    else if( error == -5 || error == -6 || error == -7 || error == -8 || error == 5 || error == 6 || error == 7 || error == 8){
        moveForward(20);
        stopAllMotors();
        readSensors();
        analogToDigital();
        error = calcError();
    }
    mazeSolver(error);   
}

  void moveForward(int t){
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, LOW);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, LOW);
    delay(t);
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, HIGH);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, HIGH);
    }

/*
 * Program to brake all motors
 */

 void stopAllMotors(){
    digitalWrite(M1_PIN1, HIGH);
    digitalWrite(M1_PIN2, HIGH);
    digitalWrite(M2_PIN1, HIGH);
    digitalWrite(M2_PIN2, HIGH);
    delay(10);
  }
  void testAllMotor(){
    int DURATION = 200;
    for(int i=1;i<10;i++){
      for(int j=0;j<DURATION;j++){
        bothMotorOnForward(i);
      }
    }
    for(int i=1;i<10;i++){
      for(int j=0;j<DURATION;j++){
        bothMotorOnReverse(i);
      }
    }
     for(int i=1;i<10;i++){
      for(int j=0;j<DURATION;j++){
        leftMotorOn(i);
      }
    }
    for(int i=1;i<10;i++){
      for(int j=0;j<DURATION;j++){
        rightMotorOn(i);
      }
    }
  }


  void UTurn(){
    //Serial.println("UTurn");
    if(pathIndex < MAX_ARRAY_SIZE)
        path[pathIndex++] = 'U';
     stopAllMotors();
     rotateCloclwiseOnAxis(150);
      int angle = 150;
      do{
      rotateCloclwiseOnAxis(10);
      angle +=  10;
      readSensors();
      analogToDigital();
      if(input[2] == 1)
        break;
      }while(angle<200);
}
void mazeSolver(int error){
  
    int dir = 0;
      /*
       *  MAZE SOLVER
       *  
       *  WHITE PATH DIR - RIGHT & LEFT BOTH 
       *  DECISION - LEFT OR RIGHT ?
       *  
       *  
       */
    if(error == 56)
          UTurn();
    else if( error == -5 || error == -6 || error == -7 || error == -8 ){//Left turn present
          moveForward(400);
          stopAllMotors();
          readSensors();
          analogToDigital();
          error = calcError();
          if(error != 56){//its a intersection - it has L,S
            if(pathIndex >= LENGTH)
              return;
            dir = path[pathIndex++];
            if(dir == 'S')
                return;
          }
          
          int angle = 70;
          rotateAntiCloclwiseOnAxis(70);
          while(angle<110){
            readSensors();
            analogToDigital();
             if(input[2] == 1)
                break;
            rotateAntiCloclwiseOnAxis(10);
            angle +=  10;
          }
    }else if(error == 64){//intersection point : L,R
          moveForward(400);
          stopAllMotors();
          
          readSensors();
          analogToDigital();
          error = calcError();
          if(error == 64){
            algorithm();
            implPhase = false;
            moveForward(400);
            stopAllMotors();
            return;
          }
          if(pathIndex >= LENGTH)
              return;
            dir = path[pathIndex++];
            if(dir == 'L'){
                int angle = 70;
                rotateAntiCloclwiseOnAxis(70);
                while(angle<110){
                  readSensors();
                  analogToDigital();
                   if(input[2] == 1)
                      break;
                  rotateAntiCloclwiseOnAxis(10);
                  angle +=  10;
                }
            }else if(dir == 'R'){
                int angle = 70;
                rotateCloclwiseOnAxis(70);
                while(angle<110){
                  readSensors();
                  analogToDigital();
                   if(input[2] == 1)
                      break;
                  rotateCloclwiseOnAxis(10);
                  angle +=  10;
                }
            }
          //Serial.println("L");
    }else if( error == 5 || error == 6 || error == 7 || error == 8 ){//Right turn present
          moveForward(400);
          stopAllMotors();
          readSensors();
          analogToDigital();
          error = calcError();
          if(error != 56){//its a intersection - it has R,S
            if(pathIndex >= LENGTH)
              return;
            dir = path[pathIndex++];
            if(dir == 'S')
              return;
          }
          int angle = 70;
          rotateCloclwiseOnAxis(70);
          while(angle<110){
            readSensors();
            analogToDigital();
             if(input[2] == 1)
                break;
            rotateCloclwiseOnAxis(10);
            angle +=  10;
          }
    //Serial.println("R");
    }
       
}

int calcError(){
  int error = 0;
         if( input[4]==0 && input[3]==0 && input[2]==0 && input[1]==0 && input[0]==0 )
    error=56;                         //special error codes
    else if( input[4]==1 && input[3]==1 && input[2]==1 && input[1]==1 && input[0]==1 )
    error=64;                         //special error codes
    else if( input[4]==0 && input[3]==1 && input[2]==1 && input[1]==0 && input[0]==1 )
    error=8;
    else if( input[4]==0 && input[3]==1 && input[2]==0 && input[1]==0 && input[0]==1 )
    error=7;
    else if( input[4]==0 && input[3]==1 && input[2]==1 && input[1]==1 && input[0]==1 )
    error=6;
    else if( input[4]==0 && input[3]==0 && input[2]==1 && input[1]==1 && input[0]==1 )
    error=5;
    else if( input[4]==0 && input[3]==0 && input[2]==0 && input[1]==0 && input[0]==1 )
    error=4;
    else if( input[4]==0 && input[3]==0 && input[2]==0 && input[1]==1 && input[0]==1 )
    error=3;
    else if( input[4]==0 && input[3]==0 && input[2]==0 && input[1]==1 && input[0]==0 )
    error=2;
    else if( input[4]==0 && input[3]==0 && input[2]==1 && input[1]==1 && input[0]==0 )
    error=1;
    else if( input[4]==0 && input[3]==0 && input[2]==1 && input[1]==0 && input[0]==0 )
    error=0;
    else if( input[4]==0 && input[3]==1 && input[2]==1 && input[1]==0 && input[0]==0 )
    error=-1;
    else if( input[4]==0 && input[3]==1 && input[2]==0 && input[1]==0 && input[0]==0 )
    error=-2;
    else if( input[4]==1 && input[3]==1 && input[2]==0 && input[1]==0 && input[0]==0 )
    error=-3;
    else if( input[4]==1 && input[3]==0 && input[2]==0 && input[1]==0 && input[0]==0 )
    error=-4;
    else if( input[4]==1 && input[3]==1 && input[2]==1 && input[1]==0 && input[0]==0 )
    error=-5;
    else if( input[4]==1 && input[3]==1 && input[2]==1 && input[1]==1 && input[0]==0 )
    error=-6;
    else if( input[4]==1 && input[3]==0 && input[2]==0 && input[1]==1 && input[0]==0 )
    error=-7;
    else if( input[4]==1 && input[3]==0 && input[2]==1 && input[1]==1 && input[0]==0 )
    error=-8;
    
  return error;
  }


void readSensors(){
  sensors[0] = analogRead(A0);
  sensors[1] = analogRead(A1);
  sensors[2] = analogRead(A2);
  sensors[3] = analogRead(A3);
  sensors[4] = analogRead(A4);  
}


/*
 * 
 * It bsicaally calculates the average of all the sensors and uses it as threshold.
 * If the range between all inputs is between 200 then it assigns 0 to all the digital inputs.
 * 
 */
void analogToDigital(){
  
  int avg=0,i=0;
  int least=1024,highest=0;
  for(i=0;i<5;i++){
     input[i] = 0;
     avg += sensors[i];
     if(least > sensors[i])
        least = sensors[i];
     if(highest < sensors[i])
        highest = sensors[i];
  }

  if(highest - least > 200)
    avg/=5;// range > 500 means some r white n some r black, hence calc new avg
  else
    avg = last_avg;// range < 500, all sensors r on either white or black... so we need to use last average
    
  last_avg = avg;
  for(i=0;i<5;i++)
       if(sensors[i] <= avg)
          input[i] = 1;
  
}
void algorithm(){
  int k = 0; 
    for(int i=0;i<pathIndex-2;i++){
      if(path[i+1] == 'U'){
        if( path[i] == 'S' && path[i+2] == 'L' )
            path[k++] = 'R';
        if( path[i] == 'L' && path[i+2] == 'L' )
            path[k++] = 'S';
        i+=2;
      }else
        path[k++] = path[i];
    }
    pathIndex = k;
}

void setup() {
  pinMode(M1_PIN1, OUTPUT);
  pinMode(M1_PIN2, OUTPUT);
  pinMode(M2_PIN1, OUTPUT);
  pinMode(M2_PIN2, OUTPUT);
  readPath();
  pinMode(13, OUTPUT);//TESTING PURPOSE
  digitalWrite(13,HIGH);
  delay(3000);
  digitalWrite(13,LOW);  
  //Serial.begin(9600);
  //Serial.println("Starttttttttt");
}

void loop() {
  if(implPhase){
    readSensors();
    analogToDigital();
    int error=calcError();
    motorController(error);
  }

}

