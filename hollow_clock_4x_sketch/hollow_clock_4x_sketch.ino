// esp8266 内存 过小 废弃


#include <string.h>

// #include <Arduino_JSON.h>

#include <Arduino.h>
#include <DNSServer.h>
#include <FS.h>

#if defined(ESP8266)

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>

#define SPIFFS LittleFS

#endif
#if defined(ESP32)

#include <SPIFFS.h>
#include <WiFi.h>
#include <WebServer.h>

#endif

#include <ESP8266TimerInterrupt.h> //需要加载Esp8266TimerInterrupt库，by Khoi Hoang
#include <ESP8266_ISR_Timer.h>
#include <ESP8266_ISR_Timer.hpp>


#include <NTPClient.h>
#include <WiFiUdp.h>

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

// #include <ArduinoOTA.h>

// #include "AudioFileSourceHTTPStream.h"
// #include "AudioFileSource.h"
// #include "AudioFileSourceBuffer.h"
// #include "AudioGeneratorMP3.h"
// #include "AudioOutputI2SNoDAC.h"

#define FIRMWARE_VERSION 10101

// 悬浮时钟 开启 debug 日志 在最终版本中注释掉
#define HOLLOW_CLOCK_DEBUG
// 串口 波特率
#define SERIAL_BAUD_RATE 115200
// 停止按钮引脚
#define STOP_BUTTON_PIN 14      // GPIO14
// 6点触发 引脚
#define HOUR_IN_PIN 12          // GPIO12
// 整点触发 引脚
#define MINUTE_IN_PIN 13        // GPIO13
// 步进电机引脚
#define STEPPER_A_PIN 15        // GPIO15
#define STEPPER_B_PIN 5         // GPIO5
#define STEPPER_C_PIN 0         // GPIO0
#define STEPPER_D_PIN 4         // GPIO4
// 步进电机每一步delay时长 单位：ms
#define STEPPER_DELAY_TIME 1
// wifi ap 名称
#define WIFI_AP_NAME "Hollow_Clock_4X_"
// #define WIFI_AP_PASSWD "hollow_clock_4x"
#define WIFI_AP_PASSWD "12345678"
#define WIFI_AP_IP 192,168,4,1

#define WIFI_IP_DOMAIN "hollow_clock_4x.local"

#define DNS_PORT 53
#define HTTP_PORT 80

#define USING_TIM_DIV1 true // for shortest and most accurate timer
#define USING_TIM_DIV16 false // for medium time and medium accurate timer
#define USING_TIM_DIV256 false // for longest timer but least accurate
#define TIMER_FREQ_HZ 1000 // 定时器触发频率

#define TIMER_HANDLER_FREQ 4 // 定义 定时器分频的最大数量

// Motor and clock parameters
// 4096 * 90 / 12 = 30720
// 时钟 一小时 走的步数
#define STEPS_PER_ROTATION 30720 // steps for a full turn of minute rotor

// 时钟模式
#define CLOCK_MODE_SECOND 0
#define CLOCK_MODE_MINUTE 1


#define CLOCK_ZERO_POS_INTERVAL_MIN 10000
#define CLOCK_ZERO_POS_INTERVAL_MAX 60000

// 默认时区
#define DEFAULT_TIME_ZONE 8

// #define CLOCK_ZERO_POS_INTERVAL_MIN 60000
// #define CLOCK_ZERO_POS_INTERVAL_MAX 60 * 60 * 1000

ESP8266Timer ITimer;

// ESP-12E 引脚映射
// static const uint8_t D0   = 3;
// static const uint8_t D1   = 1;
// static const uint8_t D2   = 16;
// static const uint8_t D3   = 5;
// static const uint8_t D4   = 4;
// static const uint8_t D5   = 14;
// static const uint8_t D6   = 12;
// static const uint8_t D7   = 13;
// static const uint8_t D8   = 0;
// static const uint8_t D9   = 2;
// static const uint8_t D10  = 15;
// static const uint8_t D11  = 13;
// static const uint8_t D12  = 12;
// static const uint8_t D13  = 14;
// static const uint8_t D14  = 4;
// static const uint8_t D15  = 5;

// =========================================
// define global variables
// =========================================

// 当前程序运行时间戳
volatile unsigned long current_timestamp;

// 网络时间 时间戳
volatile unsigned long network_timestamp;

// 当前程序运行时间
volatile unsigned long current_millis;


//设置需要连接的wifi的名称和密码
const char *ssid = "ChinaNet-KmSz";
const char *password ="19910529575";

// AudioGeneratorMP3 *mp3;
// AudioFileSourceHTTPStream *file;
// AudioFileSourceBuffer *buff;
// AudioOutputI2SNoDAC *out;

// =========================================
// define function
// =========================================

void Serial_Loginfo(String str){
#ifdef HOLLOW_CLOCK_DEBUG
  Serial.println("[INFO] " + str);
#endif
}

void Serial_Logdebug(String str){
#ifdef HOLLOW_CLOCK_DEBUG
  Serial.println("[DEBUG] " + str);
#endif
}

// 保存 millis 不能用在中断中
void save_run_millis(unsigned long m = NULL){
  current_millis = m == NULL ? millis() : m;
}




// =========================================
// define class
// =========================================

class Stepper {
protected:
  int port[4];
  int seq[8][4] = {
    {  LOW, HIGH, HIGH,  LOW},
    {  LOW,  LOW, HIGH,  LOW},
    {  LOW,  LOW, HIGH, HIGH},
    {  LOW,  LOW,  LOW, HIGH},
    { HIGH,  LOW,  LOW, HIGH},
    { HIGH,  LOW,  LOW,  LOW},
    { HIGH, HIGH,  LOW,  LOW},
    {  LOW, HIGH,  LOW,  LOW}
  };
  int delaytime = 1;

  // 是否运行 flag
  volatile bool running = false;

  // 是否在异步执行
  volatile bool _async = false;

  // 异步执行时 的总步数
  volatile int _async_step_cnt = 0;

  // 异步执行时 的当前步数
  volatile int _async_crt_step_cnt = 0;

  // 异步执行时 方向  false: 正向 true:反向
  volatile bool _async_direction = false;

  void (*_async_cb) ();

  volatile int phase = 0;

private:
  void clearAsyncFlag();

public:
  Stepper();
  Stepper(uint8_t a_pin, uint8_t b_pin, uint8_t c_pin, uint8_t d_pin);
  Stepper(uint8_t a_pin, uint8_t b_pin, uint8_t c_pin, uint8_t d_pin, int delay_time);
  void rotate(int step);
  void rotateAsync(int step, void (* cb) ());
  void addRotateAsync(int step);
  void rotateAsyncHandle();
  void stop();
  bool isRunning();
  bool isAsync();
};

Stepper::Stepper(){}

Stepper::Stepper(uint8_t a_pin, uint8_t b_pin, uint8_t c_pin, uint8_t d_pin){
  this->port[0] = a_pin;
  this->port[1] = b_pin;
  this->port[2] = c_pin;
  this->port[3] = d_pin;

  pinMode(this->port[0], OUTPUT);
  pinMode(this->port[1], OUTPUT);
  pinMode(this->port[2], OUTPUT);
  pinMode(this->port[3], OUTPUT);

  this->delaytime = STEPPER_DELAY_TIME;
}

Stepper::Stepper(uint8_t a_pin, uint8_t b_pin, uint8_t c_pin, uint8_t d_pin, int delay_time){
  this->port[0] = a_pin;
  this->port[1] = b_pin;
  this->port[2] = c_pin;
  this->port[3] = d_pin;

  pinMode(this->port[0], OUTPUT);
  pinMode(this->port[1], OUTPUT);
  pinMode(this->port[2], OUTPUT);
  pinMode(this->port[3], OUTPUT);

  this->delaytime = 1;
}

// 偏转角度 同步便转角度 会阻塞
void Stepper::rotate(int step) {
  this->running = true;

  static int phase = 0;
  int i, j;
  int delta = (step < 0) ? 1 : 7;

  step = (step >= 0) ? step : -step;
  for(j = 0; j < step; j++) {
    phase = (phase + delta) % 8;
    for(i = 0; i < 4; i++) {
      digitalWrite(this->port[i], this->seq[phase][i]);
    }
    delay(this->delaytime);
  }
  this->stop();
};

// 偏转角度 异步执行 需和 rotateAsyncHandle 配合使用
void Stepper::rotateAsync(int step, void (* cb) () = NULL) {

  this->stop();

  this->running = true;
  this->_async = true;

  // 记录 步数
  this->_async_step_cnt = step >= 0 ? step : -step;
  this->_async_crt_step_cnt = 0;

  // 是否反向
  this->_async_direction = step < 0;

  this->_async_cb = cb;

};

void Stepper::addRotateAsync(int step) {

  if(this->running && this->_async){

    int tmp_step = step >= 0 ? step : -step;

    // 如果 正在 正转
    if(!this->_async_direction){
      // 如果 步数 > 0 说明增加正转 直接 在 电机步数上 增加
      if(step >= 0){
        this->_async_step_cnt += tmp_step;
      }else{
        // 如果 步数 < 0  说明减小反转
        if(this->_async_step_cnt - this->_async_crt_step_cnt >= tmp_step){
          this->_async_step_cnt -= tmp_step;
        }else{
           this->_async_step_cnt = this->_async_step_cnt - this->_async_crt_step_cnt;
           this->_async_crt_step_cnt  = 0;
           this->_async_direction = !this->_async_direction;
        }
      }
    }else{
      // 如果正在反转
      if(step >= 0){
         // 如果 步数 < 0  说明减小反转
        if(this->_async_step_cnt - this->_async_crt_step_cnt >= tmp_step){
          this->_async_step_cnt -= tmp_step;
        }else{
           this->_async_step_cnt = this->_async_step_cnt - this->_async_crt_step_cnt;
           this->_async_crt_step_cnt  = 0;
           this->_async_direction = !this->_async_direction;
        }
      }else{
       this->_async_step_cnt += tmp_step;
      }
    }
  }
};

// 偏转角度 异步处理
void Stepper::rotateAsyncHandle() {

  if(this->running && this->_async){

  //     Serial_Logdebug("[Stepper] running: " + String(this->running)
  // + " async: " + String(this->_async)
  // + " _async_crt_step_cnt: " + String(this->_async_crt_step_cnt)
  // + " _async_step_cnt" + String(this->_async_step_cnt));


    // 正在运行 且是异步 ，还有 需要执行的步数
    if(this->_async_crt_step_cnt < this->_async_step_cnt){

      int delta = (this->_async_direction) ? 1 : 7;
      this->phase = (this->phase + delta) % 8;

      // Serial_Logdebug("[Stepper] phase: " + String(phase) + " delta: " + String(delta));

      for(int i = 0; i < 4; i++) {
        digitalWrite(this->port[i], this->seq[phase][i]);
      }

      // 每执行一步 记录下来
      this->_async_crt_step_cnt++;

    }else{
      if(this->_async_cb != NULL){
        this->_async_cb();
      }
      this->stop();
    }
  }

};

void Stepper::stop(){
  // power cut
  for(int i = 0; i < 4; i++) {
    digitalWrite(this->port[i], LOW);
  }
  this->running = false;

  if(this->_async){
    this->clearAsyncFlag();
  }
}

bool Stepper::isRunning(){
  return this->running;
}

bool Stepper::isAsync(){
  return this->_async;
}

void Stepper::clearAsyncFlag(){
  this->_async = false;
  this->_async_step_cnt = 0;
  this->_async_crt_step_cnt = 0;
  this->_async_cb = NULL;
  this->phase = 0;
}


// =========================================
// define variables
// =========================================


// 步进电机对象
Stepper *stepper;

// 是否正在播放音频 default: false
volatile bool audio_is_running = false;

// 是否正在校对时间 default: false
volatile bool prood_time_is_runnning = false;

// 6点触发中断 锁 记录上次触发时间
volatile unsigned long hour_in_lock = 0;

// 整点触发中断 锁 记录上次触发时间
volatile unsigned long minute_in_lock = 0;

// 停止按钮中断 锁 记录上次触发时间
volatile unsigned long stop_btn_lock = 0;

// 6点触发中断 是否激活
volatile unsigned long hour_in_enable = 0;
volatile unsigned long minute_in_enable = 0;

// 时钟频率数组 值为多少毫秒
int time_handler_frequency[TIMER_HANDLER_FREQ];
int time_handler_frequency_len = 0;
void (* time_handler_list[TIMER_HANDLER_FREQ]) ();
unsigned long time_handler_counter = -1;             // 频率为 1000 时，相当于记录运行的毫秒数
unsigned long time_handler_freq_previous_time[TIMER_HANDLER_FREQ];

// 建立网络服务器对象，该对象用于响应HTTP请求。监听端口（80）
ESP8266WebServer httpd_server(HTTP_PORT);

// 建立DNS服务
DNSServer dns_server;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp.aliyun.com"); //NTP地址

volatile bool ntp_inited = true;
// 时间允许获取 flag
volatile bool ntp_allow_update_flag = false;

// wifi是否已连接
volatile bool wifi_connected_notify = false;

// 时钟模式
// 0: 秒模式 一次走一秒钟的距离
// 1: 分模式 一次走一分钟的距离
volatile int clock_mode = 0;

// 时区
volatile int time_zone = DEFAULT_TIME_ZONE;

// 是否在运行 时间校对
volatile bool proof_time_inited = false;
volatile bool proof_time_running = false;


// =========================================
// define interrupt
// =========================================

// 当按下 停止 按钮时，执行此中断处理
void ICACHE_RAM_ATTR stop_btn_interrupt(){
  if(current_millis - stop_btn_lock < 500 && current_millis - stop_btn_lock > 0) return;
  stop_btn_lock = current_millis;
  Serial_Loginfo("触发 停止按钮 中断.");


  // 检查是否在这个播放 音频如果有立刻打断

}

// 当 触发 6时 时，执行此中断处理
void ICACHE_RAM_ATTR hour_in_interrupt(){
  // 每次触发时，记录触发时的时间
  hour_in_enable = current_millis;
  if(current_millis - hour_in_lock < CLOCK_ZERO_POS_INTERVAL_MIN && current_millis - hour_in_lock > 0) return;
  hour_in_lock = current_millis;
  Serial_Loginfo("触发 6点 中断...");

  // 检查是否6点报告给主程序


  // 如果初始化过， 且 一小时内 再次 触发跳过
  if(proof_time_inited && current_millis - hour_in_lock <  CLOCK_ZERO_POS_INTERVAL_MAX && current_millis - hour_in_lock > 0){
    Serial_Loginfo("自动对时已初始化, 跳过");
    return;
  }



  // 如果正在对时， 计算 时间偏差，计算需要走的步数， 开始对时

  // current_millis

  // current_timestamp

  // 获取 时分，计算 距离 6点 的位置，6点 到 12点之间， 正转， 0点到6点之间， 反转
  // STEPS_PER_ROTATION / 60    秒需要转的步数

  // 计算 需要转的圈数 如果 大于 12点 时 减 12 小时
  // 先 停止掉 步进电机

  Serial_Loginfo("停掉 步进电机");
  stepper->stop();


  Serial_Loginfo("计算 偏移量");

  int hours = get_hours_from_current_timestamp();
  int minutes = get_minutes_from_current_timestamp();
  int seconds = get_seconds_from_current_timestamp();

  // 转为 12小时制
  hours = hours >= 12 ? hours - 12 : hours;

  int pos_point_time = 6;

  // 小时 间隔
  int hours_range = hours - pos_point_time;

  Serial_Loginfo("current time:  " + String(hours) + ":" + String(minutes) + ":" + String(seconds));

  int tmp_step = 0;
  //  6点 到 12点之间 正转 指定的 圈数
  if(hours_range >= 0){
    tmp_step = STEPS_PER_ROTATION * hours_range + ((STEPS_PER_ROTATION / 60) * minutes) +  ((STEPS_PER_ROTATION / 60 / 60) * seconds);

    Serial_Loginfo("6点 到 12点之间 正转 " + String(hours_range) + " " + String(minutes) + " " + String(seconds) + " steps:" + String(tmp_step));
    Serial_Loginfo("目标时间: " + String(hours) + ":" + String(minutes) + ":" + String(seconds));

  }else{

    hours_range = -hours_range;

    int pos_seconds = (60 - seconds) % 60;

    int pos_minutes = pos_seconds > 0
                    ? (60 - minutes - 1) % 60
                    : (60 - minutes) % 60;

    int pos_hours   = pos_point_time - hours - 1;

    tmp_step = STEPS_PER_ROTATION * pos_hours + ((STEPS_PER_ROTATION / 60) * pos_minutes) +  ((STEPS_PER_ROTATION / 60 / 60) * pos_seconds);

    tmp_step = -tmp_step;

    Serial_Loginfo("0点 到 6点之间 反转 " + String(pos_hours) + " " + String(pos_minutes) + " " + String(pos_seconds) + " steps:" + String(tmp_step));
    Serial_Loginfo("目标时间: " + String(hours) + ":" + String(minutes) + ":" + String(seconds));
  }

  stepper->rotateAsync(tmp_step, prood_time_after);

}

// 当 触发 整点 时，执行此中断处理
void ICACHE_RAM_ATTR min_in_interrupt(){
  // 每次触发时，记录触发时的时间
  minute_in_enable = current_millis;
  if(current_millis - minute_in_lock < CLOCK_ZERO_POS_INTERVAL_MIN && current_millis - minute_in_lock > 0) return;
  minute_in_lock = current_millis;
  Serial_Loginfo("触发 整点 中断...");

}

bool is_hour_in_enable(){
  return hour_in_enable > 0
    && current_millis - hour_in_enable > 0
    && current_millis - hour_in_enable < 1500;
}

bool is_minute_in_enable(){
  return minute_in_enable > 0
    && current_millis - minute_in_enable > 0
    && current_millis - minute_in_enable < 1500;
}



// 定时器，执行此中断处理
void ICACHE_RAM_ATTR TimerHandler() {

  time_handler_counter++;
  save_run_millis(time_handler_counter);

  for(int i = 0; i < time_handler_frequency_len; i++){
    if(time_handler_counter % time_handler_frequency[i] == 0){
      time_handler_list[i]();
    }
  }

}

void timer_1ms_handler(){

  // Serial_Loginfo("1ms");

  // 步进电机每1ms检查一次是否有异步任务需要处理
  stepper->rotateAsyncHandle();

  // Serial_Loginfo("1ms end");

  // 如果 6点 整
  // if(is_hour_in_enable() && is_minute_in_enable()){
  //   hollow_clock_zero_pos_notify();
  // }

}

// 时钟 0 定位 点 通知
// void hollow_clock_zero_pos_notify(){

//   // 当 到达 6点整 如果 现在在自动校时， 获取当前时间，和计算6点到 当前时间的偏差 进行

//   Serial_Loginfo("hollow_clock_zero_pos_notify");

// }



void timer_1000ms_handler(){
  int timer_val = 1000;
  // Serial_Loginfo("timer_1000ms_handler    1000ms");

  ntp_allow_update_flag = true;

  // 每秒 当前 时间戳 加 1
  current_timestamp++;

  timer_check_wifi_connect_status();

  timer_clock_interval_handle(timer_val);

}

// 五分钟 定时任务
void timer_300000ms_handler(){

  // stepper->rotateAsync(STEPS_PER_ROTATION/60);
  if(ntp_inited && WiFi.status() == WL_CONNECTED){
    current_timestamp = network_timestamp;
  }

}

void timer_60000ms_handler(){
  int timer_val = 60000;

  timer_clock_interval_handle(timer_val);

}


// 处理 时钟每次移动的步数
void timer_clock_interval_handle(int timer_val){

// #ifdef HOLLOW_CLOCK_DEBUG

//     if(!stepper->isRunning()){
//       stepper->rotateAsync(0);
//     }
//     stepper->addRotateAsync(STEPS_PER_ROTATION / 60);

// #else

  if(clock_mode == CLOCK_MODE_SECOND && timer_val == 1000){

    if(!stepper->isRunning()){
      stepper->rotateAsync(0);
    }
    stepper->addRotateAsync(STEPS_PER_ROTATION / 60 / 60);
    int crt_second = get_seconds_from_current_timestamp();
    if(crt_second == 0){
      // 补偿 每分钟缺少 32 步的问题
      // STEPS_PER_ROTATION / 60 / 60 = 8.533333333333333
      // 512 - (60 * 8 = 480) = 32
      stepper->addRotateAsync(32);
      // stepper->addRotateAsync((STEPS_PER_ROTATION / 60) - (60 * (STEPS_PER_ROTATION / 60 / 60)));
    }

  }else if(clock_mode == CLOCK_MODE_MINUTE && timer_val == 60000){

    if(!stepper->isRunning()){
      stepper->rotateAsync(0);
    }
    stepper->addRotateAsync(STEPS_PER_ROTATION / 60);

  }

// #endif

}

void timer_check_wifi_connect_status(){

  //如果wifi连接成功，WiFi.status()返回值则为，WL_CONNECTED
  if (WiFi.status() == WL_CONNECTED && !wifi_connected_notify){
    wifi_connected_notify = true;
    wifi_connect_success_notify();
  }

}

// wifi 连接成功 通知 函数
void wifi_connect_success_notify() {

  Serial_Loginfo("Wi-Fi 连接成功");
  Serial_Loginfo("您的开发板的IP:");
  //当连接成功之后，向串口输出开发板的ip地址
  Serial_Loginfo(WiFi.localIP().toString());


  // 开始 校时
  start_proof_time();

  // 获取 网络时间
  get_network_time_after_wifi_connect();

  // 初始化 OTA
  // setup_arduino_ota();

}

void get_network_time_after_wifi_connect(){

  ntp_inited = false;

}

// 获取网络时间处理函数
void get_network_time_handler() {

  if(!ntp_inited){
    timeClient.end();
    timeClient.begin();
    timeClient.setUpdateInterval(5 * 60 * 1000);    // 五分钟 更新同步一次网络时间
    timeClient.setTimeOffset(3600 * time_zone); //+1区，偏移3600，+8区，偏移3600*8
  }

  if(!ntp_allow_update_flag || WiFi.status() != WL_CONNECTED){ return; }
  // 执行完后将标记置为false
  ntp_allow_update_flag = false;

  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();

  network_timestamp = epochTime;

  // 如果 ntp 首次初始化， 初始化完毕后， 设置网络时间为当前时间
  if(!ntp_inited){
    ntp_inited = true;
    current_timestamp = network_timestamp;
  }

  // // 打印时间
  // int currentHour = timeClient.getHours();
  // int currentMinute = timeClient.getMinutes();
  // int weekDay = timeClient.getDay();

  // //将epochTime换算成年月日
  // struct tm *ptm = gmtime((time_t *)&epochTime);

  // int monthDay = ptm->tm_mday;
  // int currentMonth = ptm->tm_mon + 1;
  // int year = ptm->tm_year;

  // Serial.printf("Epoch Time: %d current time: %d-%d-%d %d:%d\n", epochTime, (year + 1900), currentMonth, monthDay, currentHour, currentMinute);

}

int get_year_from_current_timestamp(){
  struct tm *ptm = gmtime((time_t *)&current_timestamp);
  return ptm->tm_year + 1900;
}
int get_month_from_current_timestamp(){
  struct tm *ptm = gmtime((time_t *)&current_timestamp);
  return ptm->tm_mon + 1;
}
int get_month_day_from_current_timestamp(){
  struct tm *ptm = gmtime((time_t *)&current_timestamp);
  return ptm->tm_mday;
}

int get_hours_from_current_timestamp(){
  // struct tm *ptm = gmtime((time_t *)&current_timestamp);
  // return ptm->tm_hour;
  return timeClient.getHours();
}

int get_minutes_from_current_timestamp(){
  // struct tm *ptm = gmtime((time_t *)&current_timestamp);
  // return ptm->tm_min;
  return timeClient.getMinutes();
}

int get_seconds_from_current_timestamp(){
  // struct tm *ptm = gmtime((time_t *)&current_timestamp);
  // return ptm->tm_sec;
   return timeClient.getSeconds();
}





// 初始化 串口
void setup_serial(){
  Serial_Loginfo("Init 串口");
  Serial.begin(SERIAL_BAUD_RATE);
}

// 初始化变量
void setup_init_variables(){

  // 设置为 当前时间确保可以在一开始触发事件
  stop_btn_lock = current_millis;
  hour_in_lock = current_millis;
  minute_in_lock = current_millis;
}

// 初始化 定时器
void setup_timer(){

  Serial_Loginfo("Init 定时器");

  ITimer.attachInterrupt(TIMER_FREQ_HZ, TimerHandler);

  time_handler_frequency[0] = 1;
  time_handler_list[0] = timer_1ms_handler;

  time_handler_frequency[1] = 1000;                        // 1秒
  time_handler_list[1] = timer_1000ms_handler;

    time_handler_frequency[2] = 60000;                    // 5分钟
  time_handler_list[2] = timer_60000ms_handler;

      time_handler_frequency[3] = 300000;                 // 5分钟
  time_handler_list[3] = timer_300000ms_handler;

  time_handler_frequency_len = 4;

  Serial_Loginfo("Init 定时器 finish");

}



// 初始化 步进电机
void setup_stepper(){

  Serial_Loginfo("Init 步进电机");

  stepper = new Stepper(STEPPER_A_PIN, STEPPER_B_PIN, STEPPER_C_PIN, STEPPER_D_PIN);

}

// 初始化 停止按钮
void setup_stop_button(){
  Serial_Loginfo("Init 停止按钮");

    //设置中断触发程序
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(STOP_BUTTON_PIN, stop_btn_interrupt, FALLING);

}

// 初始化 霍尔传感器输入
void setup_hall_sensor(){
  Serial_Loginfo("Init 霍尔传感器输入");

  //设置中断触发程序
  pinMode(HOUR_IN_PIN, INPUT_PULLUP);
  attachInterrupt(HOUR_IN_PIN, hour_in_interrupt, FALLING);
  // attachInterrupt(HOUR_IN_PIN, hour_in_interrupt_after, RISING);

  pinMode(MINUTE_IN_PIN, INPUT_PULLUP);
  attachInterrupt(MINUTE_IN_PIN, min_in_interrupt, FALLING);
  // attachInterrupt(MINUTE_IN_PIN, min_in_interrupt_after, RISING);

}

// 初始化 音频输出
void setup_audio(){
  Serial_Loginfo("Init 音频输出");

}

// 初始化 SPIFFS
void setup_spiffs(){
  Serial_Loginfo("Init SPIFFS");

  bool mount = SPIFFS.begin();

  if(!mount){
    Serial_Loginfo("[SPIFFS] mount failed");
  }

  FSInfo fsinfo;
  SPIFFS.info(fsinfo);

  Serial_Loginfo("[SPIFFS] totalBytes\t\t" + String(fsinfo.totalBytes));
  Serial_Loginfo("[SPIFFS] usedBytes\t\t" + String(fsinfo.usedBytes));
  Serial_Loginfo("[SPIFFS] blockSize\t\t" + String(fsinfo.blockSize));
  Serial_Loginfo("[SPIFFS] pageSize\t\t" + String(fsinfo.pageSize));
  Serial_Loginfo("[SPIFFS] maxOpenFiles\t\t" + String(fsinfo.maxOpenFiles));
  Serial_Loginfo("[SPIFFS] maxPathLength\t" + String(fsinfo.maxPathLength));

}

// 初始化 wifi ap
void setup_wifi_ap(){
  Serial_Loginfo("Init Wi-Fi AP");

  // 设置内网
  IPAddress softLocal(WIFI_AP_IP);   // 1 设置内网WIFI IP地址
  IPAddress softGateway(WIFI_AP_IP);
  IPAddress softSubnet(255,255,255,0);
  WiFi.softAPConfig(softLocal, softGateway, softSubnet);

  // 2 设置WIFI名称
  String apName = (WIFI_AP_NAME + (String)ESP.getChipId());
  const char *softAPName = apName.c_str();

  // 3创建wifi  名称 +密码 如果没有密码不设置
  #ifdef WIFI_AP_PASSWD
  WiFi.softAP(softAPName, WIFI_AP_PASSWD);
  #else
  WiFi.softAP(softAPName);
  #endif
  // 4输出创建的WIFI IP地址
  IPAddress myIP = WiFi.softAPIP();

  // 5输出WIFI 名称
  Serial_Loginfo("[Wi-Fi] softAPName: ");
  Serial_Loginfo(apName);
  Serial_Loginfo("AP IP address: ");
  Serial_Loginfo(myIP.toString());

  Serial_Loginfo("Init Wi-Fi AP Finish");

}


// 初始化 dns server
void setup_dnsserver(){
  Serial_Loginfo("Init DNS Server");

  /* Setup the DNS server redirecting all the domains to the apIP */
  dns_server.setErrorReplyCode(DNSReplyCode::NoError);
  dns_server.start(DNS_PORT, "*", WiFi.softAPIP());

}


// 首页处理
void httpd_handler_index(){

  Serial_Logdebug("[httpd] /index.html 200");

  String path = "/index.html";
  String contentType = getContentType(path); // 获取文件类型
  if (SPIFFS.exists(path)) {                          // 如果访问的文件可以在SPIFFS中找到
      File file = SPIFFS.open(path, "r");             // 则尝试打开该文件
      httpd_server.streamFile(file, contentType);     // 并且将该文件返回给浏览器
      file.close();                                   // 并且关闭文件
      return;                                         // 返回true
  }else{
    httpd_server.send(200, "text/html", "<h1>Hollow Clock 4X</h1> <p>未初始化</p>");
  }

}

void httpd_handler_init_html(){
  httpd_server.send(200,
  "text/html",
  "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\"><title>Clock Init</title><meta charset=\"utf-8\"><meta http-equiv=\"X-UA-Compatible\"content=\"IE=edge\"><meta name=\"viewport\"content=\"width=device-width, initial-scale=1\"></head><body><textarea class=\"init_text_input\"style=\"width: 100%;height: 200px;\"value=\"\"></textarea><div class=\"submit\">提交</div><script type=\"application/javascript\">document.onreadystatechange=function(){document.querySelector(\".submit\").onclick=async function(){var val=document.querySelector(\".init_text_input\").value;var val_items=val.trim().split(\"\n\");for(var i=0;i<val_items.length;i++){var val_item=val_items[i].trim();var val_arr=val_item.split(\" \");var val_path=val_arr[0].trim();var val_url=val_arr[1].trim();var resp=await fetch('./api/init_file_upload?path='+val_path+'&url='+val_url,{method:'get',headers:{'Content-Type':'text/plain'}}).then((res)=>res.text());console.log(resp)}}}</script></body></html>"
  );
}

// 初始化
void httpd_handler_init_file_upload(){
  String path = httpd_server.arg("path");
  String url = httpd_server.arg("url");
  Serial_Loginfo(path + " " + url);

  WiFiClient wifiClient;
  HTTPClient httpClient;

  //重点2 通过begin函数配置请求地址。此处也可以不使用端口号和PATH而单纯的
  httpClient.begin(wifiClient, url);
  Serial.print("URL: ");
  Serial.println(url);

  //重点3 通过GET函数启动连接并发送HTTP请求
  int httpCode = httpClient.GET();
  Serial.print("Send GET request to URL: ");
  Serial.println(url);

  String r = "";

  //重点4. 如果服务器响应HTTP_CODE_OK(200)则从服务器获取响应体信息并通过串口输出
  //如果服务器不响应HTTP_CODE_OK(200)则将服务器响应状态码通过串口输出
  if (httpCode == HTTP_CODE_OK) {
    // 使用getString函数获取服务器响应体内容
    WiFiClient& stream = httpClient.getStream();

    if(stream.available()){
      Serial.println("stream.available()");

      // 打开文件
      File file = SPIFFS.open(path, "w");

      Serial.println("file availableForWrite " + file.availableForWrite());

      uint8_t buf[256];
      int len = -1;
      while((len = stream.readBytes(buf, 128)) > 0){
        file.write(buf, len);
      }
      file.flush();
      // 关闭文件
      file.close();

      Serial.println("file exists " + String(SPIFFS.exists(path)));


       r = "OK";
    }else{
      Serial.println("Stream Unavailable");
      r = "Stream Unavailable";
    }


  } else {
    r = "FAILED " + String(httpCode);
  }

  //重点5. 关闭ESP8266与服务器连接
  httpClient.end();
  wifiClient.stop();

  httpd_server.send(200, "text/plain", path + " " + r);

}

// 处理用户浏览器的HTTP访问
void httpd_handler_other() {

    // 获取用户请求网址信息
    String webAddress = httpd_server.uri();


    // 通过handleFileRead函数处处理用户访问
    bool fileReadOK = httpd_handler_file_read(webAddress);

    // 如果在SPIFFS无法找到用户访问的资源，则回复404 (Not Found)
    if (!fileReadOK) {
      Serial_Loginfo(("[httpd] 404 " + (String)httpd_server.hostHeader() + (String)webAddress));
      // httpd_server.send(404, "text/plain", "404 Not Found");
      httpd_handler_captive_portal();
    }

}

// 回复状态码 200 给客户端
void httpd_handler_respond_ok() {
    httpd_server.send(200);
}

bool httpd_handler_file_read(String path) { // 处理浏览器HTTP访问

    if (path.endsWith("/")) {                         // 如果访问地址以"/"为结尾
        path = "/index.html";                         // 则将访问地址修改为/index.html便于SPIFFS访问
    }

    String contentType = getContentType(path); // 获取文件类型

    if (SPIFFS.exists(path)) {                        // 如果访问的文件可以在SPIFFS中找到
        File file = SPIFFS.open(path, "r");           // 则尝试打开该文件
        httpd_server.streamFile(file, contentType); // 并且将该文件返回给浏览器
        file.close();                                 // 并且关闭文件
        return true;                                  // 返回true
    }
    return false; // 如果文件未找到，则返回false

}


/** IP to String? */
String toStringIp(IPAddress ip) {
  String res = "";
  for (int i = 0; i < 3; i++) {
    res += String((ip >> (8 * i)) & 0xFF) + ".";
  }
  res += String(((ip >> 8 * 3)) & 0xFF);
  return res;
}

bool isIp(String str) {
  for (size_t i = 0; i < str.length(); i++) {
    int c = str.charAt(i);
    if (c != '.' && (c < '0' || c > '9')) {
      return false;
    }
  }
  return true;
}

// 实现强制跳转门户网站
bool httpd_handler_captive_portal() {
  if (!isIp(httpd_server.hostHeader()) ) {
    Serial_Loginfo("Request redirected to captive portal");
    // httpd_server.sendHeader("Location", String("http://") + toStringIp(httpd_server.client().localIP()), true);

    httpd_server.sendHeader("Location", String("http://") + WIFI_IP_DOMAIN, true);

    httpd_server.send(302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
    httpd_server.client().stop(); // Stop is needed because we sent no content length
    return true;
  }
  return false;
}

// 获取文件类型
String getContentType(String filename) {
    if (filename.endsWith(".htm"))
        return "text/html";
    else if (filename.endsWith(".html"))
        return "text/html";
    else if (filename.endsWith(".css"))
        return "text/css";
    else if (filename.endsWith(".js"))
        return "application/javascript";
    else if (filename.endsWith(".png"))
        return "image/png";
    else if (filename.endsWith(".gif"))
        return "image/gif";
    else if (filename.endsWith(".jpg"))
        return "image/jpeg";
    else if (filename.endsWith(".ico"))
        return "image/x-icon";
    else if (filename.endsWith(".xml"))
        return "text/xml";
    else if (filename.endsWith(".pdf"))
        return "application/x-pdf";
    else if (filename.endsWith(".zip"))
        return "application/x-zip";
    else if (filename.endsWith(".gz"))
        return "application/x-gzip";
    return "text/plain";

}

// 初始化 web server
void setup_httpd(){
  Serial_Loginfo("Init WebServer");

  httpd_server.on("/", httpd_handler_index);         // Index

  httpd_server.on("/init.html", httpd_handler_init_html);         // Index

  // 初始化 文件
  httpd_server.on("/api/init_file_upload", httpd_handler_init_file_upload);         // Index

  // httpd_server.on("/upload.html",                    // 如果客户端通过upload页面
  //                   HTTP_POST,                         // 向服务器发送文件(请求方法POST)
  //                   respondOK,                         // 则回复状态码 200 给客户端
  //                   handleFileUpload);                 // 并且运行处理文件上传函数

  httpd_server.onNotFound(httpd_handler_other);


  httpd_server.begin(); // 启动网站服务

  Serial_Loginfo("HTTP server started");

}


// 初始化 wifi 连接
void setup_wifi_connect(){
  Serial_Loginfo("Init Wi-Fi 连接");

  // //连接WIFI
  WiFi.begin(ssid,password);

}



bool GetLocalTime(struct tm * info, uint32_t ms) {
  uint32_t count = ms / 10;
  time_t now;

  time(&now);
  localtime_r(&now, info);

  if (info->tm_year > (2016 - 1900)) {
    return true;
  }

  while (count--) {
    delay(10);
    time(&now);
    localtime_r(&now, info);
    if (info->tm_year > (2016 - 1900)) {
      return true;
    }
  }
  return false;
}

// 初始化 网络时间
void setup_net_time(){
  // Serial_Loginfo("Init 网络时间");

  // while(WiFi.status() != WL_CONNECTED){
  //   delay(1000);
  // }

  // 放置到 wifi 连接以后
  // timeClient.begin();
  // timeClient.setTimeOffset(28800); //+1区，偏移3600，+8区，偏移3600*8

}

// 初始化 校对时间
void setup_proof_time(){
  // Serial_Loginfo("Init 时间校对");


  // Serial_Loginfo("run stepper.");
  // stepper->rotateAsync(10000000);
}

// void setup_arduino_ota(){
//   Serial_Loginfo("Init Arduino OTA");

//   ArduinoOTA.onStart([]() {
//     String type;
//     if (ArduinoOTA.getCommand() == U_FLASH) {
//       type = "sketch";
//     } else { // U_FS
//       type = "filesystem";
//     }

//     // NOTE: if updating FS this would be the place to unmount FS using FS.end()
//     Serial_Loginfo("[ArduinoOTA] Start updating " + String(type));
//   });
//   ArduinoOTA.onEnd([]() {
//     Serial_Loginfo("\n[ArduinoOTA] End");
//   });
//   ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
//     Serial_Loginfo("[ArduinoOTA] Progress: " + String((progress / (total / 100))) + "%\r");
//   });
//   ArduinoOTA.onError([](ota_error_t error) {
//     Serial_Loginfo("[ArduinoOTA] Error[" + String(error) + "]: ");
//     if (error == OTA_AUTH_ERROR) {
//       Serial_Loginfo("[ArduinoOTA] Auth Failed");
//     } else if (error == OTA_BEGIN_ERROR) {
//       Serial_Loginfo("[ArduinoOTA] Begin Failed");
//     } else if (error == OTA_CONNECT_ERROR) {
//       Serial_Loginfo("[ArduinoOTA] Connect Failed");
//     } else if (error == OTA_RECEIVE_ERROR) {
//       Serial_Loginfo("[ArduinoOTA] Receive Failed");
//     } else if (error == OTA_END_ERROR) {
//       Serial_Loginfo("[ArduinoOTA] End Failed");
//     }
//   });

//   ArduinoOTA.begin();

//   Serial.println("ArduinoOTA Ready");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.localIP());

// }

// 开始自动校时
void start_proof_time(){

  Serial_Loginfo("Init 时间校对");

  proof_time_inited = false;
  proof_time_running = true;

  stepper->rotateAsync(STEPS_PER_ROTATION * 24);

}

// 自动校时结束
void prood_time_after(){
  proof_time_inited = true;
  proof_time_running = false;

  int hours = get_hours_from_current_timestamp();
  int minutes = get_minutes_from_current_timestamp();
  int seconds = get_seconds_from_current_timestamp();

  Serial_Loginfo("时间校对完毕");
  Serial_Loginfo("当前时间: " + String(hours) + ":" + String(minutes) + ":" + String(seconds));
}

// 固件升级
void firmware_upgrade_started() {
  Serial_Loginfo("CALLBACK:  HTTP update process started");
}

void firmware_upgrade_finished() {
  Serial_Loginfo("CALLBACK:  HTTP update process finished");
}

void firmware_upgrade_progress(int cur, int total) {
  Serial_Loginfo("CALLBACK:  HTTP update process at "+ String(cur)+" of " + String(total)+ " bytes...");
}

void firmware_upgrade_error(int err) {
  Serial_Loginfo("CALLBACK:  HTTP update fatal error code " + String(err));
}

// 固件升级
String firmware_upgrade(String binfile){
  WiFiClient client;
  t_httpUpdate_return ret = ESPhttpUpdate.update(client, binfile);
  // Or:
  //t_httpUpdate_return ret = ESPhttpUpdate.update(client, "server", 80, "file.bin");

  String r = "";
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      r = "HTTP_UPDATE_FAILD Error (" + String(ESPhttpUpdate.getLastError())+ "): " + ESPhttpUpdate.getLastErrorString().c_str();
      break;

    case HTTP_UPDATE_NO_UPDATES:
      r = "HTTP_UPDATE_NO_UPDATES";
      break;

    case HTTP_UPDATE_OK:
      r = "HTTP_UPDATE_OK";
      break;
  }
  Serial_Loginfo(r);
  return r;
}

// 初始化固件升级
void setup_firmware_upgrade(){

    // The line below is optional. It can be used to blink the LED on the board during flashing
    // The LED will be on during download of one buffer of data from the network. The LED will
    // be off during writing that buffer to flash
    // On a good connection the LED should flash regularly. On a bad connection the LED will be
    // on much longer than it will be off. Other pins than LED_BUILTIN may be used. The second
    // value is used to put the LED on. If the LED is on with HIGH, that value should be passed
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);

    // Add optional callback notifiers
    ESPhttpUpdate.onStart(firmware_upgrade_started);
    ESPhttpUpdate.onEnd(firmware_upgrade_finished);
    ESPhttpUpdate.onProgress(firmware_upgrade_progress);
    ESPhttpUpdate.onError(firmware_upgrade_error);

}


// 初始化
void setup() {

  // 记录当前运行时间
  save_run_millis();

  // 初始化 serial
  setup_serial();

  delay(1000);

  setup_init_variables();

  // 初始化 步进电机
  setup_stepper();

  // 初始化 stop 按钮
  setup_stop_button();

  // 初始化 霍尔传感器输入
  setup_hall_sensor();

  // 初始化 音频 输出
  setup_audio();

  // 初始化spiffs
  setup_spiffs();

  // 初始化 定时器
  setup_timer();

  // 初始化 wifi  AP/站点
  setup_wifi_ap();

  // 初始化 DNSServer
  setup_dnsserver();

  // 初始化 web server
  setup_httpd();

  // 初始化 wifi 连接
  setup_wifi_connect();

  // 初始化 网络时间
  setup_net_time();

  // 初始化 时间校对
  setup_proof_time();

  // 初始化 固件升级
  setup_firmware_upgrade();

  Serial_Loginfo("setup finished.");

}

// 主循环
void loop() {

  // unsigned long crt_millis = millis();
  // unsigned long prv_millis = current_millis;
  // save_run_millis(crt_millis);

  // 获取时间
  get_network_time_handler();

  dns_server.processNextRequest();

  httpd_server.handleClient();

  // ArduinoOTA.handle();

  // Serial_Loginfo("Running Time: " + String(current_millis));

}



