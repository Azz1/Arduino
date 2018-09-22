 //============================智宇科技===========================
//  智能小车黑线循、红外避障、遥控综合实验
//===============================================================
//#include <Servo.h> 
#include <IRremote.h>//包含红外库  关键点
#include <LiquidCrystal.h> //申明1602液晶的函数库
//申明1602液晶的引脚所连接的Arduino数字端口，8线或4线数据模式，任选其一
//LiquidCrystal lcd(12,11,10,9,8,7,6,5,4,3,2);   //8数据口模式连线声明
LiquidCrystal lcd(13,12,7,6,5,4,3); //4数据口模式连线声明 P13--LCD 4脚  P12--LCD 5脚  
              //P7--LCD 6脚   P6--LCD 11脚  P5--LCD 12脚  P4--LCD 13脚  P3--LCD 14脚  

int Echo = A1;  // Echo回声脚(P2.0)
int Trig =A0;  //  Trig 触发脚(P2.1)

int Front_Distance = 0;//
int Left_Distance = 0;
int Right_Distance = 0;

int RECV_PIN = A4;//端口声明
IRrecv irrecv(RECV_PIN);
decode_results results;//结构声明
int on = 0;//标志位
unsigned long last = millis();

long run_car = 0x00FF629D;//按键CH
long back_car = 0x00FFA857;//按键+
long left_car = 0x00FF22DD;//按键<<
long right_car = 0x00FFC23D;//按键>||
long stop_car = 0x00FF02FD;//按键>>|
long left_turn = 0x00ffE01F;//按键-
long right_turn = 0x00FF906F;//按键EQ
long previous_motion = 0l;
int in_void = 0;

int Left_motor=8;     //左电机(IN3) 输出0  前进   输出1 后退
int Left_motor_pwm=9;     //左电机PWM调速

int Right_motor_pwm=10;    // 右电机PWM调速
int Right_motor=11;    // 右电机后退(IN1)  输出0  前进   输出1 后退

#define PORT_KEY  A2     //定义按键 数字A2 接口
#define PORT_LED1  7    //定义LED 数字7 接口
#define PORT_LED2  12   //定义LED2 数字12 接口
int beep=A3;//定义蜂鸣器 数字A3 接口

#define KEYMODE_1   1  // 定义按键模式1
#define KEYMODE_2   2  // 定义按键模式2
#define KEYMODE_3   3  // 定义按键模式3
uint8_t keyMode;      //指无符号8bit整型数
const int SensorRight = 3;   	//右循迹红外传感器(P3.2 OUT1)
const int SensorLeft = 4;     	//左循迹红外传感器(P3.3 OUT2)
const int SensorRight_2 = 5;   	//左边红外避障传感器()
const int SensorLeft_2 = 6;   	//右边红外避障传感器()

int SR_2;    //右边红外避障传感器状态
int SL_2;    //左边红外避障传感器状态
int SL;    //左循迹红外传感器状态
int SR;    //右循迹红外传感器状态

int servopin=2;//设置舵机驱动脚到数字口2
int myangle;//定义角度变量
int pulsewidth;//定义脉宽变量

void setup()
{
  //初始化电机驱动IO为输出方式
  pinMode(Left_motor,OUTPUT); // PIN 8 8脚无PWM功能
  pinMode(Left_motor_pwm,OUTPUT); // PIN 9 (PWM)
  pinMode(Right_motor_pwm,OUTPUT);// PIN 10 (PWM) 
  pinMode(Right_motor,OUTPUT);// PIN 11 (PWM)
  pinMode(beep,OUTPUT);

  pinMode(SensorRight, INPUT); //定义右循迹红外传感器为输入
  pinMode(SensorLeft, INPUT); //定义左循迹红外传感器为输入
  pinMode(SensorRight_2, INPUT); //定义右红外传感器为输入
  pinMode(SensorLeft_2, INPUT); //定义右红外传感器为输入
  pinMode(13, OUTPUT);////端口模式，输出
  Serial.begin(9600);	//波特率9600
  irrecv.enableIRIn(); // Start the receiver 这个函数对寻迹  壁障的PWM控制有影响  右转函数
  
  KeyScanInit();
  LEDInit();

  //初始化超声波引脚
  pinMode(Echo, INPUT);    // 定义超声波输入脚
  pinMode(Trig, OUTPUT);   // 定义超声波输出脚
  lcd.begin(16,2);      //初始化1602液晶工作                       模式
  //定义1602液晶显示范围为2行16列字符  
  pinMode(servopin,OUTPUT);//设定舵机接口为输出接口
}

//=======================智能小车的基本动作=========================
//void run(int time)     // 前进
void run()     // 前进
{
  digitalWrite(Right_motor,LOW);  // 右电机前进
  digitalWrite(Right_motor_pwm,HIGH);  // 右电机前进     
  analogWrite(Right_motor_pwm,255);//PWM比例0~255调速，左右轮差异略增减
   
  digitalWrite(Left_motor,HIGH);  // 左电机前进
  digitalWrite(Left_motor_pwm,HIGH);  //左电机PWM     
  analogWrite(Left_motor_pwm,255);//PWM比例0~255调速，左右轮差异略增减
  //delay(time * 100);   //执行时间，可以调整  
}

void back(int time)          //后退
{
  digitalWrite(Right_motor,HIGH);  // 右电机后退
  digitalWrite(Right_motor_pwm,HIGH);  // 右电机前进     
  analogWrite(Right_motor_pwm,255);//PWM比例0~255调速，左右轮差异略增减
  
  digitalWrite(Left_motor,LOW);  // 左电机后退
  digitalWrite(Left_motor_pwm,HIGH);  //左电机PWM     
  analogWrite(Left_motor_pwm,255);//PWM比例0~255调速，左右轮差异略增减
  delay(time * 100);   //执行时间，可以调整    
}

void brake(int time)         //刹车，停车
{
  digitalWrite(Right_motor_pwm,LOW);  // 右电机PWM 调速输出0      
  analogWrite(Right_motor_pwm,0);//PWM比例0~255调速，左右轮差异略增减

  digitalWrite(Left_motor_pwm,LOW);  //左电机PWM 调速输出0          
  analogWrite(Left_motor_pwm,0);//PWM比例0~255调速，左右轮差异略增减
  delay(time * 100);//执行时间，可以调整  
}

void left(int time)         //左转(左轮不动，右轮前进)
{
  digitalWrite(Right_motor,LOW);  // 右电机前进
  digitalWrite(Right_motor_pwm,HIGH);  // 右电机前进     
  analogWrite(Right_motor_pwm,255);//PWM比例0~255调速，左右轮差异略增减
  
  digitalWrite(Left_motor,LOW);  // 左电机前进
  digitalWrite(Left_motor_pwm,LOW);  //左电机PWM     
  analogWrite(Left_motor_pwm,0);//PWM比例0~255调速，左右轮差异略增减
  delay(time * 100);	//执行时间，可以调整  
}

void spin_left(int time)         //左转(左轮后退，右轮前进)
{
  digitalWrite(Right_motor,LOW);  // 右电机前进
  digitalWrite(Right_motor_pwm,HIGH);  // 右电机前进     
  analogWrite(Right_motor_pwm,255);//PWM比例0~255调速，左右轮差异略增减
  
  digitalWrite(Left_motor,LOW);  // 左电机后退
  digitalWrite(Left_motor_pwm,HIGH);  //左电机PWM     
  analogWrite(Left_motor_pwm,255);//PWM比例0~255调速，左右轮差异略增减
  delay(time * 100);	//执行时间，可以调整  
}

void right(int time)        //右转(右轮不动，左轮前进)
{
  digitalWrite(Right_motor,LOW);  // 右电机不转
  digitalWrite(Right_motor_pwm,LOW);  // 右电机PWM输出0     
  analogWrite(Right_motor_pwm,0);//PWM比例0~255调速，左右轮差异略增减
  
  digitalWrite(Left_motor,HIGH);  // 左电机前进
  digitalWrite(Left_motor_pwm,HIGH);  //左电机PWM     
  analogWrite(Left_motor_pwm,255);//PWM比例0~255调速，左右轮差异略增减
  delay(time * 100);	//执行时间，可以调整  
}

void spin_right(int time)        //右转(右轮后退，左轮前进)
{
  digitalWrite(Right_motor,HIGH);  // 右电机后退
  digitalWrite(Right_motor_pwm,HIGH);  // 右电机PWM输出1     
  analogWrite(Right_motor_pwm,255);//PWM比例0~255调速，左右轮差异略增减
  
  digitalWrite(Left_motor,HIGH);  // 左电机前进
  digitalWrite(Left_motor_pwm,HIGH);  //左电机PWM     
  analogWrite(Left_motor_pwm,255);//PWM比例0~255调速，左右轮差异略增减
  delay(time * 100);	//执行时间，可以调整    
}

//==========================================================


void dump(decode_results *results)
{
  int count = results->rawlen;
  if (results->decode_type == UNKNOWN) 
  {
    //Serial.println("Could not decode message");
    brake(2);
  } 
}

void Robot_Avoidance()                   //机器人避障子程序
{
   //有信号为LOW  没有信号为HIGH
    SR_2 = digitalRead(SensorRight_2);
    SL_2 = digitalRead(SensorLeft_2);
    if (SL_2 == HIGH && SR_2==HIGH)
    {
        run();   //调用前进函数
        digitalWrite(beep,LOW);
    }
    else if (SL_2 == HIGH && SR_2 == LOW)// 右边探测到有障碍物，有信号返回，向左转 
      left(2);
    else if (SR_2 == HIGH && SL_2 == LOW) //左边探测到有障碍物，有信号返回，向右转  
      right(2);
    else // 都是有障碍物, 后退
    {
        digitalWrite(beep,HIGH);		//蜂鸣器响
         //digitalWrite(LED,HIGH);		//LED亮
         brake(3);//停止200MS
         back(8);//后退500MS
         left(5);//调用左转函数  延时500ms 
    }
}

void Robot_Avoidance_norun()                   //机器人避障子程序
{
   //有信号为LOW  没有信号为HIGH
    SR_2 = digitalRead(SensorRight_2);
    SL_2 = digitalRead(SensorLeft_2);
    digitalWrite(beep,LOW);

    if (previous_motion != stop_car && previous_motion != back_car) {
      front_detection();//测量前方距离
      if(Front_Distance < 30)//当遇到障碍物时
      {
        brake(2);//先刹车
        back(4);//后退减速
        brake(2);//停下来做测距
        left_detection();//测量左边距障碍物距离
        Distance_display(Left_Distance);//液晶屏显示距离
        right_detection();//测量右边距障碍物距离
        Distance_display(Right_Distance);//液晶屏显示距离
        if((Left_Distance < 30 ) &&( Right_Distance < 30 ))//当左右两侧均有障碍物靠得比较近
        {
          back(1);
          spin_left(2);//旋转掉头
         }
        else if(Left_Distance > Right_Distance)//左边比右边空旷
        {      
          left(15);//左转
          brake(1);//刹车，稳定方向
        }
        else//右边比左边空旷
        {
          right(15);//右转
          brake(1);//刹车，稳定方向
        }
      }
    }

/*      
    if (SL_2 == HIGH && SR_2 == LOW)// 右边探测到有障碍物，有信号返回，向左转 
    {
        left(12);
        in_void = 1; 
    }
    else if (SR_2 == HIGH && SL_2 == LOW) //左边探测到有障碍物，有信号返回，向右转  
    {
        right(12);
        in_void = 1; 
    }
    else if (SL_2 == LOW && SR_2 == LOW) // 都是有障碍物, 后退
    {
        digitalWrite(beep,HIGH);    //蜂鸣器响
        //digitalWrite(LED,HIGH);    //LED亮
        brake(3);//停止200MS
        back(16);//后退500MS
        left(12);//调用左转函数  延时500ms 
        in_void = 1; 
    }
  */
    
    if( in_void > 0 ) {
      in_void = 0;
      if (previous_motion == run_car ) //按键CH
        run();//前进
      if (previous_motion == back_car )//按键+
        back(2);//后退
      if (previous_motion == left_car )//按键<<
        left(2);//左转
      if (previous_motion == right_car )//按键>||
        right(2);//右转
      if (previous_motion == stop_car )//按键>>|
        brake(2);//停车
      if (previous_motion == left_turn )//按键-
        spin_left(2);//左旋转
      if (previous_motion == right_turn )//按键EQ
        spin_right(2);//右旋转
    }
}

void Robot_Traction()                     //机器人循迹子程序
{
   //有信号为LOW  没有信号为HIGH
  SR = digitalRead(SensorRight);//有信号表明在白色区域，车子底板上L1亮；没信号表明压在黑线上，车子底板上L1灭
  SL = digitalRead(SensorLeft);//有信号表明在白色区域，车子底板上L2亮；没信号表明压在黑线上，车子底板上L2灭
  if (SL == LOW && SR==LOW)
    run();   //调用前进函数
  else if (SL == HIGH && SR == LOW)// 左循迹红外传感器,检测到信号，车子向右偏离轨道，向左转 
    spin_left(2);
  else if (SR == HIGH && SL == LOW) // 右循迹红外传感器,检测到信号，车子向左偏离轨道，向右转  
    spin_right(2);
  else // 都是白色, 停止
    brake(2);
}

void IR_IN()                             //机器人遥控子程序
{
  Robot_Avoidance_norun();
  
  if (irrecv.decode(&results)) //调用库函数：解码
  {
    if (millis() - last > 250) //确定接收到信号
    {
      on = !on;//标志位置反
      digitalWrite(13, on ? HIGH : LOW);//板子上接收到信号闪烁一下led
      dump(&results);//解码红外信号
    }

    previous_motion = results.value;
    
    if (results.value == run_car ) //按键CH
      run();//前进
    if (results.value == back_car )//按键+
      back(2);//后退
    if (results.value == left_car )//按键<<
      left(2);//左转
    if (results.value == right_car )//按键>||
      right(2);//右转
    if (results.value == stop_car )//按键>>|
      brake(2);//停车
    if (results.value == left_turn )//按键-
      spin_left(2);//左旋转
    if (results.value == right_turn )//按键EQ
      spin_right(2);//右旋转
    last = millis();      
    irrecv.resume(); // Receive the next value
  }
}

// 按键处理初始化
void KeyScanInit(void)
{
    pinMode(PORT_KEY,INPUT_PULLUP); //输入模式，内部上拉。
    keyMode = KEYMODE_1;
}

// 任务：按键处理
void KeyScanTask(void)//
{
    static uint8_t keypre = 0; //按键被按下时置1.
    if( (keypre == 0) && (digitalRead(PORT_KEY) == HIGH) ) //按键被按下
    {
        keypre = 1; //置1，避免持续按下按键时再次进入此函数体。
        switch(keyMode)
        {
        case KEYMODE_1:
            keyMode = KEYMODE_2;  //关键点错位赋值
            break;
        case KEYMODE_2:
            keyMode = KEYMODE_3;
            break;
        case KEYMODE_3:
            keyMode = KEYMODE_1;
            break;
        default:
            break;
        }
    }
    if(digitalRead(PORT_KEY) == LOW) //按键被放开
    {
        keypre = 0; //置0，允许再次切换LED模式
    }
}

// LED初始化
void LEDInit(void)
{
    pinMode(PORT_LED1,OUTPUT);
    pinMode(PORT_LED2,OUTPUT);
    digitalWrite(PORT_LED1,LOW);
    digitalWrite(PORT_LED2,LOW);
}

// 任务：循迹、避障、遥控模式处理
void LEDTask(void)
{
    switch(keyMode)
    {
    case KEYMODE_1:
        digitalWrite(PORT_LED1,HIGH);
        digitalWrite(PORT_LED2,LOW);
        IR_IN();    //调用遥控子程序
        break;
    case KEYMODE_2:
        digitalWrite(PORT_LED1,LOW);
        digitalWrite(PORT_LED2,HIGH);
        Robot_Traction(); //调用循迹子程序
        break;
    case KEYMODE_3:
        digitalWrite(PORT_LED1,HIGH);
        digitalWrite(PORT_LED2,HIGH);
        Robot_Avoidance(); // 调用避障子程序
        break;
    default:
        break;
    }
}

float Distance_test()   // 量出前方距离 
{
  digitalWrite(Trig, LOW);   // 给触发脚低电平2μs
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  // 给触发脚高电平10μs，这里至少是10μs
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);    // 持续给触发脚低电
  float Fdistance = pulseIn(Echo, HIGH);  // 读取高电平时间(单位：微秒)
  Fdistance= Fdistance/58;       //为什么除以58等于厘米，  Y米=（X秒*344）/2
  // X秒=（ 2*Y米）/344 ==》X秒=0.0058*Y米 ==》厘米=微秒/58
  //Serial.print("Distance:");      //输出距离（单位：厘米）
  //Serial.println(Fdistance);         //显示距离
  //Distance = Fdistance;
  return Fdistance;
}  

void Distance_display(int Distance)//显示距离
{
  if((2<Distance)&(Distance<400))
  {
    lcd.home();        //把光标移回左上角，即从头开始输出   
    lcd.print("    Distance: ");       //显示
    lcd.setCursor(6,2);   //把光标定位在第2行，第6列
    lcd.print(Distance);       //显示距离
    lcd.print("cm");          //显示
  }
  else
  {
    lcd.home();        //把光标移回左上角，即从头开始输出  
    lcd.print("!!! Out of range");       //显示
  }
  delay(250);
  lcd.clear();
}

void servopulse(int servopin,int myangle)/*定义一个脉冲函数，用来模拟方式产生PWM值舵机的范围是0.5MS到2.5MS 1.5MS 占空比是居中周期是20MS*/ 
{
  pulsewidth=(myangle*11)+500;//将角度转化为500-2480 的脉宽值 这里的myangle就是0-180度  所以180*11+50=2480  11是为了换成90度的时候基本就是1.5MS
  digitalWrite(servopin,HIGH);//将舵机接口电平置高                                      90*11+50=1490uS  就是1.5ms
  delayMicroseconds(pulsewidth);//延时脉宽值的微秒数  这里调用的是微秒延时函数
  digitalWrite(servopin,LOW);//将舵机接口电平置低
 // delay(20-pulsewidth/1000);//延时周期内剩余时间  这里调用的是ms延时函数
  delay(20-(pulsewidth*0.001));//延时周期内剩余时间  这里调用的是ms延时函数
}

void front_detection()
{
  //此处循环次数减少，为了增加小车遇到障碍物的反应速度
  for(int i=0;i<=5;i++) //产生PWM个数，等效延时以保证能转到响应角度
  {
    servopulse(servopin,90);//模拟产生PWM
  }
  Front_Distance = Distance_test();
  //Serial.print("Front_Distance:");      //输出距离（单位：厘米）
 // Serial.println(Front_Distance);         //显示距离
 //Distance_display(Front_Distance);
}

void left_detection()
{
  for(int i=0;i<=15;i++) //产生PWM个数，等效延时以保证能转到响应角度
  {
    servopulse(servopin,175);//模拟产生PWM
  }
  Left_Distance = Distance_test();
  //Serial.print("Left_Distance:");      //输出距离（单位：厘米）
  //Serial.println(Left_Distance);         //显示距离
}

void right_detection()
{
  for(int i=0;i<=15;i++) //产生PWM个数，等效延时以保证能转到响应角度
  {
    servopulse(servopin,5);//模拟产生PWM
  }
  Right_Distance = Distance_test();
  //Serial.print("Right_Distance:");      //输出距离（单位：厘米）
  //Serial.println(Right_Distance);         //显示距离
}

void loop()
{ 
  while(1)
  {
    KeyScanTask();
    LEDTask();
  }  
}


