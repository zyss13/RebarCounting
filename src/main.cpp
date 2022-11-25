#include <Arduino.h>

const int time = 15;  //每次正反转的时间 

const int pin2 = 4;  //pin 4、5 为电机1路信号引脚
const int pin3 = 5;

const int jg_pin = 6;   //激光发射头引脚
const int jetsonPin = 9; //控制jetson继电器
const int st_pin1 = 10;  // 左激光测距引脚
const int st_pin2 = 11;  // 右激光测距引脚

const int Left = 2; //左红外对射中断引脚
const int Right = 3; //右红外对射中断引脚


unsigned long xzTime1; // 旋转开始时的毫秒值时间
unsigned long xzTime2; // 旋转结束的时间
unsigned long t;    // 当前时间
int xzFlag = 1;  //0 停止 1 准备好 2正转 3 反转 20 正转中 30 反转中
int flagTemp1=1,flagTemp2=0 ;   // 中断程序中临时保存xzFlag
int st1,st2; 

// git master测试
void clearSerialCache(){  //清空Serial缓存
    while (Serial.available())
    {
        Serial.read();
    }
    
}
void istop(){ //舵机停止
 Serial.println("stop");
//  digitalWrite(pin2,HIGH);
//  digitalWrite(pin3,HIGH);
//  delay(500);
 digitalWrite(pin2,LOW);
 digitalWrite(pin3,LOW);
 delay(1000);
//  Serial.println("stop2");
//  xzFlag = 1;
//  Serial.println("stop");
}

void cw(int second){  //顺时针旋转5s停止2s

    Serial.println("cw start");
    digitalWrite(pin2,HIGH);
    digitalWrite(pin3,LOW);
    xzTime1 = millis();
    xzTime2 = xzTime1 + second*1000;
    xzFlag = 20;
}

void ac(int second){  //逆时针旋转5s停止2s
    // Serial.println("ac start");
    digitalWrite(pin2,LOW);
    digitalWrite(pin3,HIGH);
    xzTime1 = millis();
    xzTime2 = xzTime1 + second*1000;
    xzFlag = 30;
}

void jetsonBegin(){
    digitalWrite(jetsonPin,HIGH);
    digitalWrite(jg_pin,HIGH);
    // digitalWrite(jg_pin,LOW);  //该继电器低电平触发
}

void jetsonEnd(){
    digitalWrite(jetsonPin,LOW);
    digitalWrite(jg_pin,LOW);
    // digitalWrite(jg_pin,HIGH);
}

void blinkLeft(){
    int newFlagTemp1 = digitalRead(Left);
    // Serial.print("newFlagTemp1");
    // Serial.println(newFlagTemp1);
    if (newFlagTemp1==1 && flagTemp1 == 0)
    {
        istop();
        xzFlag=1;
        Serial.println("Left");
    }
    flagTemp1 = newFlagTemp1;
    
}

void blinkRight(){
    int newFlagTemp2 = digitalRead(Right);
    // Serial.print("newFlagTemp2");
    // Serial.println(newFlagTemp2);
    if (newFlagTemp2==1 && flagTemp2 == 0)
    {
        jetsonEnd();
        istop();
        // if(xzFlag == 30){
        //     xzFlag = 2;
        // }else if(xzFlag == 20){
        //     xzFlag = 3;
        // }
        xzFlag = 3;
        Serial.println("right");
    }
    flagTemp2 = newFlagTemp2;
}

void inspectLoop(){
    // Serial.print("LoopxzFlag:");
    // Serial.println(xzFlag);
    blinkLeft();
    blinkRight();
    if(xzFlag == 1) return;
    if (xzFlag == 0){
        istop();
        xzFlag = 1;
    }else if (xzFlag == 2)
    {
        cw(time);
    }else if (xzFlag == 3)
    {
        ac(time);
    }else if(xzFlag == 20){
        Serial.println("正转run..");
        // if (millis() >= xzTime2){
        //     istop();
        //     xzFlag=3;
        // }
    }else if (xzFlag == 30){
        Serial.println("反转run..");
        // if (millis() >= xzTime2){
        //     istop();
        //     xzFlag=1;
        // }
    }

    
    // delay(500);
 
}

void setup() {
  // 初始化串口
  Serial.begin(115200);
  
  pinMode(pin2,OUTPUT);
  pinMode(pin3,OUTPUT);

  pinMode(jetsonPin,OUTPUT);
  pinMode(jg_pin,OUTPUT);

  pinMode(st_pin1,INPUT_PULLUP);
  pinMode(st_pin2,INPUT_PULLUP);
  pinMode(Left,INPUT);
  pinMode(Right,INPUT);
//   digitalWrite(jg_pin,HIGH);  //激光继电器低电平触发
  digitalWrite(jetsonPin,LOW);
//   pinMode(interruptPin1,INPUT_PULLUP);
//   pinMode(interruptPin2,INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(interruptPin1),blinkLeft,RISING); //设置中断
//   attachInterrupt(digitalPinToInterrupt(interruptPin2),blinkRight,RISING); //设置中断
  Serial.println("init OK");
}

void loop() {
    
    // digitalWrite(jg_pin,HIGH);
    // Serial.println(digitalRead(jg_pin));
    // delay(2000);
    
    // digitalWrite(jg_pin,LOW);
    // Serial.println(digitalRead(jg_pin));
    // delay(2000);
    
    
    inspectLoop();
    int newst1 = digitalRead(st_pin1);
    int newst2 = digitalRead(st_pin2);
    if ( (newst1 == 1) && (st1 == 0) && (st2 == 1) ){
        xzFlag = 2;
        jetsonBegin();
        Serial.println("jetsonBegin");
    }
    st1 = newst1;
    st2 = newst2;
    delay(100); 
}