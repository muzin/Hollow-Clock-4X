// Hollow Clock 4X

#include <string.h>

// #include <Arduino_JSON.h>

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Ticker.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include <HTTPClient.h>
#include <HTTPUpdate.h>

// #include <ArduinoOTA.h>

#include "AudioFileSourceICYStream.h"
#include "AudioFileSourceBuffer.h"
#include <AudioFileSourceSPIFFS.h>
#include "AudioGeneratorMP3.h"
//#include "AudioOutputI2SNoDAC.h"
#include "AudioOutputI2S.h"

#define FIRMWARE_VERSION 10103

// 悬浮时钟 开启 debug 日志 在最终版本中注释掉
#define HOLLOW_CLOCK_DEBUG
// 串口 波特率
#define SERIAL_BAUD_RATE 115200
// 停止按钮引脚
#define STOP_BUTTON_PIN 13  // GPIO13
// 6点触发 引脚
#define HOUR_IN_PIN 14  // GPIO14
// 整点触发 引脚
#define MINUTE_IN_PIN 12  // GPIO12
// 步进电机引脚
#define STEPPER_A_PIN 16  // GPIO16
#define STEPPER_B_PIN 17  // GPIO17
#define STEPPER_C_PIN 18  // GPIO18
#define STEPPER_D_PIN 19  // GPIO19

#define BAT_ADC_PIN 35
#define BAT_VOLTAGE_MAX 3.7

// 步进电机每一步delay时长 单位：ms
#define STEPPER_DELAY_TIME 2
// wifi ap 名称
#define WIFI_AP_NAME "Hollow_Clock_4X_"
#define WIFI_AP_PASSWD "12345678"
#define WIFI_AP_IP 192, 168, 4, 1

#define WIFI_IP_DOMAIN "hc4x.fanglesoft.com"

#define DNS_PORT 53
#define HTTP_PORT 80

#define FORMAT_SPIFFS_IF_FAILED true


#define I2S_SCLK 27         // (串行时钟SCK)-->SCLK
#define I2S_LRC 33          // (字选择WS)-->LRC
#define I2S_DIN 32          // (串行数据SD)-->DIN
#define I2S_DEFAULT_GAIN 1  // 默认 音量


// Motor and clock parameters
// 4096 * 90 / 12 = 30720
// 时钟 一小时 走的步数
#define STEPS_PER_ROTATION 30720  // steps for a full turn of minute rotor

// 时钟模式
#define CLOCK_MODE_SECOND 0
#define CLOCK_MODE_MINUTE 1


#define CLOCK_ZERO_POS_INTERVAL_MIN 10000
#define CLOCK_ZERO_POS_INTERVAL_MAX 60000

// 默认时区
#define DEFAULT_TIME_ZONE 8

#define IS_12_HOUR_DEFAULT_VAL false;

// 最大闹钟数量
#define MAX_BELLS_SIZE 50

// #define CLOCK_ZERO_POS_INTERVAL_MIN 60000
// #define CLOCK_ZERO_POS_INTERVAL_MAX 60 * 60 * 1000



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

uint32_t get_esp_chipid() {
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  return chipId;
}

// =========================================
// define global variables
// =========================================


Ticker ticker1ms;       // 1ms 定时器
Ticker ticker1000ms;    // 1s 定时器
Ticker ticker20000ms;   // 20s 定时器
Ticker ticker60000ms;   // 1分钟 定时器
Ticker ticker300000ms;  // 5分钟 定时器

// audio
AudioGeneratorMP3 *mp3;
AudioFileSourceICYStream *file;
AudioFileSourceSPIFFS *spiffsfile;
AudioFileSourceBuffer *buff;
// Output device is SPDIF
AudioOutputI2S *out;

// chime audio
AudioGeneratorMP3 *chime_mp3;
AudioFileSourceICYStream *chime_file;
AudioFileSourceSPIFFS *chime_spiffsfile;
AudioFileSourceBuffer *chime_buff;
// Output device is SPDIF

// I2S 音量
// float i2s_gain = I2S_DEFAULT_GAIN;
volatile float i2s_gain = 0.2;

// 当前程序运行时间戳
volatile unsigned long current_timestamp;

// 网络时间 时间戳
volatile unsigned long network_timestamp;

// 当前程序运行时间
volatile unsigned long current_millis;


//设置需要连接的wifi的名称和密码
String wifi_ssid = "";
String wifi_pwd = "";

String wifi_ap_name = WIFI_AP_NAME + String(get_esp_chipid());
String wifi_ap_passwd = WIFI_AP_PASSWD;

// 是否正在播放音频 default: false
volatile bool audio_is_running = false;

volatile bool mp3_desync = false;

// 是否正在校对时间 default: false
volatile bool prood_time_is_runnning = false;

// 6点触发中断 锁 记录上次触发时间
volatile unsigned long hour_in_lock = 0;

// 整点触发中断 锁 记录上次触发时间
volatile unsigned long minute_in_lock = 0;

// 停止按钮中断 锁 记录上次触发时间
volatile unsigned long stop_btn_lock = 0;

// 停止按钮 是否长按。记录按下的 毫秒数 
volatile long stop_btn_long_press = -1;

// 6点触发中断 是否激活
volatile unsigned long hour_in_enable = 0;
volatile unsigned long minute_in_enable = 0;




// 建立网络服务器对象，该对象用于响应HTTP请求。监听端口（80）
WebServer httpd_server(HTTP_PORT);

// 建立DNS服务
DNSServer dns_server;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp.aliyun.com");  //NTP地址

volatile bool ntp_inited = true;
// 时间允许获取 flag
volatile bool ntp_allow_update_flag = false;

// wifi是否已连接
volatile bool wifi_connected_notify = false;
volatile int wifi_conneccted_notify_num = 0;

// 时钟模式
// 0: 秒模式 一次走一秒钟的距离
// 1: 分模式 一次走一分钟的距离
volatile int clock_mode = 0;

// 时区
volatile int time_zone = DEFAULT_TIME_ZONE;

// 是否在运行 时间校对
volatile bool proof_time_inited = false;
volatile bool proof_time_running = false;

volatile bool clock_is_12_hour = IS_12_HOUR_DEFAULT_VAL;

volatile unsigned long birthday = -1;

// 报时 mp3
String spk_time_list[] = {
  "/rings/time/t_0.mp3",
  "/rings/time/t_1.mp3",
  "/rings/time/t_2.mp3",
  "/rings/time/t_3.mp3",
  "/rings/time/t_4.mp3",
  "/rings/time/t_5.mp3",
  "/rings/time/t_6.mp3",
  "/rings/time/t_7.mp3",
  "/rings/time/t_8.mp3",
  "/rings/time/t_9.mp3",
  "/rings/time/t_10.mp3",
  "",
  "/rings/time/t_2_2.mp3",
  "/rings/time/t_bj.mp3",
  "/rings/time/t_am.mp3",
  "/rings/time/t_pm.mp3",
  "/rings/time/t_cm.mp3",
  "/rings/time/t_dot.mp3",
  "/rings/time/t_minute.mp3",
  "/rings/time/t_whole.mp3"
};

int spk_time_bj_idx = 13;
int spk_time_am_idx = 14;
int spk_time_pm_idx = 15;
int spk_time_cm_idx = 16;
int spk_time_dot_idx = 17;
int spk_time_minute_idx = 18;
int spk_time_whole_idx = 19;

String spk_not_net = "/rings/tip/no_net.mp3";
String spk_not_audio_type = "/rings/tip/not_audio_t.mp3";
String spk_not_found_audio = "/rings/tip/not_found_audio.mp3";
String spk_not_pos = "/rings/tip/not_pos.mp3";
String spk_prood_time = "/rings/tip/prood_time.mp3";
String spk_wifi_connected = "/rings/tip/wifi_connected.mp3";
String spk_hungry = "/rings/tip/hungry.mp3";
String spk_eat = "/rings/tip/eat.mp3";

String bells_prefix = "/bells";
String bells_list_info = bells_prefix + "/list.txt";
String bells_audio_list_info = bells_prefix + "/audio_list.txt";

int chime_idx_list[20];
volatile int chime_len = 0;
volatile int chime_crt = -1;
volatile int chime_crt_done = true;  // 当前 报时片段 是否结束
volatile int chime_running = false;  // 当前 报时 是否结束
volatile int chime_handling = false;

// 报时 列表
uint8_t chime_list[24][2] = { -1 };
int chime_list_len = 0;


// 音频铃声列表 
String bells_audio_list[50][2] = {"", ""};
int bells_audio_list_len = 0;

// =========================================
// define function
// =========================================

void Serial_Loginfo(String str) {
#ifdef HOLLOW_CLOCK_DEBUG
  Serial.println("[INFO] [" + get_format_datetime_from_current_timestamp() + "] " + str);
#endif
}

void Serial_Logdebug(String str) {
#ifdef HOLLOW_CLOCK_DEBUG
  Serial.println("[DEBUG] [" + get_format_datetime_from_current_timestamp() + "] " + str);
#endif
}

// 保存 millis 不能用在中断中
void save_run_millis(unsigned long m = NULL) {
  current_millis = m == NULL ? millis() : m;
}



// 重启设备
void reboot() {
  ESP.restart();
}

// =========================================
// define type
// =========================================

typedef int AudioType;
const AudioType Audio_HTTP_FILE = 1;
const AudioType Audio_SPIFFS_FILE = 2;

// =========================================
// define class
// =========================================

class Stepper {
protected:
  int port[4];
  int seq[8][4] = {
    { LOW, HIGH, HIGH, LOW },
    { LOW, LOW, HIGH, LOW },
    { LOW, LOW, HIGH, HIGH },
    { LOW, LOW, LOW, HIGH },
    { HIGH, LOW, LOW, HIGH },
    { HIGH, LOW, LOW, LOW },
    { HIGH, HIGH, LOW, LOW },
    { LOW, HIGH, LOW, LOW }
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

  void (*_async_cb)();

  volatile int phase = 0;

private:
  void clearAsyncFlag();

public:
  Stepper();
  Stepper(uint8_t a_pin, uint8_t b_pin, uint8_t c_pin, uint8_t d_pin);
  Stepper(uint8_t a_pin, uint8_t b_pin, uint8_t c_pin, uint8_t d_pin, int delay_time);
  void rotate(int step);
  void rotateAsync(int step, void (*cb)());
  void addRotateAsync(int step);
  void rotateAsyncHandle();
  void stop();
  bool isRunning();
  bool isAsync();
};

Stepper::Stepper() {}

Stepper::Stepper(uint8_t a_pin, uint8_t b_pin, uint8_t c_pin, uint8_t d_pin) {
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

Stepper::Stepper(uint8_t a_pin, uint8_t b_pin, uint8_t c_pin, uint8_t d_pin, int delay_time) {
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
  for (j = 0; j < step; j++) {
    phase = (phase + delta) % 8;
    for (i = 0; i < 4; i++) {
      digitalWrite(this->port[i], this->seq[phase][i]);
    }
    delay(this->delaytime);
  }
  this->stop();
};

// 偏转角度 异步执行 需和 rotateAsyncHandle 配合使用
void Stepper::rotateAsync(int step, void (*cb)() = NULL) {

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

  if (this->running && this->_async) {

    int tmp_step = step >= 0 ? step : -step;

    // 如果 正在 正转
    if (!this->_async_direction) {
      // 如果 步数 > 0 说明增加正转 直接 在 电机步数上 增加
      if (step >= 0) {
        this->_async_step_cnt += tmp_step;
      } else {
        // 如果 步数 < 0  说明减小反转
        if (this->_async_step_cnt - this->_async_crt_step_cnt >= tmp_step) {
          this->_async_step_cnt -= tmp_step;
        } else {
          this->_async_step_cnt = this->_async_step_cnt - this->_async_crt_step_cnt;
          this->_async_crt_step_cnt = 0;
          this->_async_direction = !this->_async_direction;
        }
      }
    } else {
      // 如果正在反转
      if (step >= 0) {
        // 如果 步数 < 0  说明减小反转
        if (this->_async_step_cnt - this->_async_crt_step_cnt >= tmp_step) {
          this->_async_step_cnt -= tmp_step;
        } else {
          this->_async_step_cnt = this->_async_step_cnt - this->_async_crt_step_cnt;
          this->_async_crt_step_cnt = 0;
          this->_async_direction = !this->_async_direction;
        }
      } else {
        this->_async_step_cnt += tmp_step;
      }
    }
  }
};

// 偏转角度 异步处理
void Stepper::rotateAsyncHandle() {

  if (this->running && this->_async) {

    //     Serial_Logdebug("[Stepper] running: " + String(this->running)
    // + " async: " + String(this->_async)
    // + " _async_crt_step_cnt: " + String(this->_async_crt_step_cnt)
    // + " _async_step_cnt" + String(this->_async_step_cnt));


    // 正在运行 且是异步 ，还有 需要执行的步数
    if (this->_async_crt_step_cnt < this->_async_step_cnt) {

      int delta = (this->_async_direction) ? 1 : 7;
      this->phase = (this->phase + delta) % 8;

      // Serial_Logdebug("[Stepper] phase: " + String(phase) + " delta: " + String(delta));

      for (int i = 0; i < 4; i++) {
        digitalWrite(this->port[i], this->seq[phase][i]);
      }

      // 每执行一步 记录下来
      this->_async_crt_step_cnt++;

    } else {
      if (this->_async_cb != NULL) {
        this->_async_cb();
      }
      this->stop();
    }
  }
};

void Stepper::stop() {
  // power cut
  for (int i = 0; i < 4; i++) {
    digitalWrite(this->port[i], LOW);
  }
  this->running = false;

  if (this->_async) {
    this->clearAsyncFlag();
  }
}

bool Stepper::isRunning() {
  return this->running;
}

bool Stepper::isAsync() {
  return this->_async;
}

void Stepper::clearAsyncFlag() {
  this->_async = false;
  this->_async_step_cnt = 0;
  this->_async_crt_step_cnt = 0;
  this->_async_cb = NULL;
  this->phase = 0;
};


enum {
  bell_report_once = 0,
  bell_report_everyday = 1,
  bell_report_custom = 2,
};

class BellItem {
private:
  String _title = "";
  String _cron = "";
  String _audio = "";
  int minute = 0;
  int hour = 0;
  int week = 0;  // 位 存储 周期
  bool enable = 0;
  int report = bell_report_once;
  bool once_finished = false;  // 只有 一次时 该值有效
public:
  BellItem() {}
  BellItem(String title, String cron, bool enable) {
    this->set_title(title);
    this->set_cron(cron);
    this->set_enable(enable);
  }
  ~BellItem() {
  }

  static void parseToBellItem(char *str, BellItem *bellItem) {
  }

  String get_title() {
    return this->_title;
  }
  void set_title(String title) {
    this->_title = title;
  }

  String get_audio() {
    return this->_audio;
  }
  void set_audio(String audio) {
    this->_audio = audio;
  }

  String get_cron() {
    return this->_cron;
  }
  void set_cron(String cron) {
    this->_cron = cron;
    String cron_str = String(cron);
    int idx_cnt = 0;
    int last_idx = 0;

    int idx = 0;

    while ((idx = cron_str.indexOf(" ", last_idx)) > -1) {
      String subs = cron_str.substring(last_idx, idx);

      // Serial_Loginfo(String("set_cron subs:") + subs);
      // Serial_Loginfo(String("set_cron idx:") + last_idx + "\t" + idx);

      if (idx_cnt == 0) {
        this->minute = subs.toInt();
      } else if (idx_cnt == 1) {
        this->hour = subs.toInt();
      } 
      last_idx = idx + 1;
      idx_cnt++;
    }

    String subs = cron_str.substring(last_idx, cron_str.length());
    this->week = subs.toInt();

    Serial_Loginfo(String("set_cron after week:") + this->week);

  }

  int get_minute() {
    return this->minute;
  }
  int get_hour() {
    return this->hour;
  }
  void set_week(int week) {
    this->week = week;
  }
  int get_week() {
    return this->week;
  }
  int get_report() {
    return this->report;
  }

  void set_report(int report) {
    this->report = report;
  }

  bool get_enable() {
    return this->enable;
  }
  void set_enable(bool enable) {
    this->enable = enable;
  }

  void set_once_finished(bool once_finished = true) {
    this->once_finished = once_finished;
  }

  bool get_once_finished() {
    return this->once_finished;
  }

  String to_string() {
    return String(this->_title) + "\n"
           + String(this->get_minute()) + " " + String(this->get_hour()) + " * * " + String(this->get_week()) + "\n"
           + String(this->get_audio()) + "\n"
           + String(this->get_report()) + "\n"
           + String(this->get_once_finished()) + "\n"
           + String(this->get_enable());
  }

  // 是否 能匹配 指定时间
  bool match(unsigned long timestamp) {
    int minute = get_minute_from_timestamp((time_t *)&timestamp);
    int hour = get_hour_from_timestamp((time_t *)&timestamp);
    int week = get_week_from_timestamp((time_t *)&timestamp);

    // Serial_Loginfo("bellitem match " + String(hour) + ":" + String(minute) + ":" + String(week));

    if (this->minute != minute) {
      return false;
    }
    if (this->hour != hour) {
      return false;
    }

    if (this->report == bell_report_once && this->once_finished == true) {
      return false;
    }

    if (this->report == bell_report_custom) {
      Serial_Loginfo("bellitem report custom: " + String(this->week) + "   (this->week >> (week - 1)) & 0x01): " + String((this->week >> (week - 1)) & 0x01));
      if (((this->week >> (week - 1)) & 0x01) == 0) {
        return false;
      }
    }

    return true;
  }

  int get_minute_from_timestamp(time_t *timestamp) {
    struct tm *ptm = gmtime(timestamp);
    return ptm->tm_min;
  }
  int get_hour_from_timestamp(time_t *timestamp) {
    struct tm *ptm = gmtime(timestamp);
    return ptm->tm_hour;
  }
  int get_week_from_timestamp(time_t *timestamp) {
    struct tm *ptm = gmtime(timestamp);
    return ptm->tm_wday;
  }
};

class BellManager {
private:
  BellItem *items[MAX_BELLS_SIZE] = { 0 };
  volatile int total = 0;
public:
  BellManager() {}

  BellItem *getByIndex(int idx) {
    if (idx > -1 && idx < this->total) {
      return this->items[idx];
    } else {
      return NULL;
    }
  }

  int get_total() {
    return total;
  }

  bool addBell(BellItem *bellItem, bool saveToFS = true) {

    String title = bellItem->get_title();

    Serial_Loginfo(String("addBell: ") + title + " " + this->existBell(title.c_str()) + " " + MAX_BELLS_SIZE);

    if (this->existBell(title.c_str())) {
      return false;
    }

    if (this->total >= MAX_BELLS_SIZE) {
      return false;
    }

    this->items[this->total] = bellItem;
    this->total++;

    if(saveToFS){
      Serial_Loginfo("saveBellToSPIFFS");

      saveBellToSPIFFS(bellItem);       
    }
    

    return true;
  }

  bool addBell(String title, String cron, String audio, bool enable, bool saveToFS = true) {
    if (this->existBell(title.c_str())) {
      return false;
    }

    if (total >= MAX_BELLS_SIZE) {
      return false;
    }

    BellItem *bellItem = new BellItem(title, cron, enable);
    bellItem->set_audio(audio);
    bellItem->set_report(bell_report_once);

    this->items[this->total] = bellItem;
    this->total++;

    if(saveToFS){
      Serial_Loginfo("saveBellToSPIFFS");

      saveBellToSPIFFS(bellItem);       
    }

    return true;
  }

  bool updateBell(String title, String new_title, String cron, String audio, bool enable, bool saveToFS = true) {
    if (!this->existBell(title.c_str())) {
      return false;
    }

    BellItem *bellItem = this->getBell(title.c_str());
    if (title != new_title) {
      bellItem->set_title(new_title);
    }
    bellItem->set_cron(cron);
    bellItem->set_audio(audio);
    bellItem->set_enable(enable);

    if(saveToFS){
      Serial_Loginfo("saveBellToSPIFFS");

      saveBellToSPIFFS(bellItem);       
    }

    return true;
  }

  bool updateBell(BellItem *bellItem, bool saveToFS = true) {
    if (!this->existBell(bellItem->get_title().c_str())) {
      return false;
    }
    if(saveToFS){
      Serial_Loginfo("saveBellToSPIFFS");

      saveBellToSPIFFS(bellItem);       
    }
    return true;
  }

  bool removeBell(const char *title, bool saveToFS = true) {
    if (!this->existBell(title)) {
      return false;
    }

    int idx = getBellIndex(title);

    BellItem *destBell = this->items[idx];

    for (int i = idx; i < total; i++) {
      if (i == total - 1) {
        this->items[i] = NULL;
      } else {
        this->items[i] = this->items[i + 1];
      }
    }

    this->total--;

    if(saveToFS){
      Serial_Loginfo("saveBellToSPIFFS");

      saveBellToSPIFFS(destBell, true);
    }
    
    delete destBell;

    return true;
  }

  void bell_handler() {

    for (int i = 0; i < this->total; i++) {
      BellItem *item = this->items[i];

      // Serial_Loginfo(item->to_string());

      if(current_timestamp < 2000){ return; }

      int second = get_seconds_from_current_timestamp();

      if (item->match(current_timestamp) && item->get_enable() && second == 0) {

        String audio_str = item->get_audio();

        if(audio_str.endsWith(".mp3")){
          start_audio(audio_str.c_str());          
        }
        
        break;
      }
    }
  }

  bool saveBellToSPIFFS(BellItem *bellItem, bool is_deleted = false) {

    File bell_list_info_file = SPIFFS.open(bells_list_info, "w");
    String cont = "";
    for (int i = 0; i < this->total; i++) {
      BellItem *item = this->items[i];
      if (i < this->total - 1) {
        cont += String(item->get_title()) + "\n";
      } else {
        cont += String(item->get_title());
      }
    }

    Serial_Loginfo(String("bell_list_info content: ") + cont.c_str());

    bell_list_info_file.write((uint8_t *)cont.c_str(), cont.length());
    bell_list_info_file.flush();
    bell_list_info_file.close();


    // 如果 是保存
    if (!is_deleted) {
      String title = bellItem->get_title();
      String bell_file_path = bells_prefix + "/" + title;
      
      File bell_file = SPIFFS.open(bell_file_path, "w");

      const char * _str = bellItem->to_string().c_str();
      bell_file.write((uint8_t *)_str, strlen(_str));
      bell_file.flush();
      bell_file.close();

      Serial_Loginfo(String("bellItem content: ") + bellItem->to_string().c_str());
      
    } else {
      String title = bellItem->get_title();
      String bell_file_path = bells_prefix + "/" + title;
      // 如果 是删除
      SPIFFS.remove(bell_file_path);
    }
  }

  bool existBell(const char *title) {
    String title_str = String(title);
    for (int i = 0; i < this->total; i++) {
      BellItem *item = this->items[i];
      if (String(item->get_title()) == title_str) {
        return true;
      }
    }
    return false;
  }

  int getBellIndex(const char *title) {
    String title_str = String(title);
    for (int i = 0; i < this->total; i++) {
      BellItem *item = this->items[i];
      if (String(item->get_title()) == title_str) {
        return i;
      }
    }
    return -1;
  }

  BellItem *getBell(const char *title) {
    int idx = this->getBellIndex(title);
    return this->items[idx];
  }
};


// =========================================
// define variables
// =========================================


// 步进电机对象
Stepper *stepper;

// 闹铃 管理器
BellManager *bellManager;


// 播放 报时
void play_chime(int hour, int minute, bool is_12_hour = false) {
  // if(is_12_hour) {
  //   hour -= 12;
  // }

  clear_chime();

  int chime_len_idx = 0;

  // 播放 现在是北京时间
  chime_idx_list[chime_len_idx++] = spk_time_bj_idx;

  // 如果 12小时制 播放 上午 下午 中午
  if (is_12_hour) {
    if (hour < 12) {
      chime_idx_list[chime_len_idx++] = spk_time_am_idx;
    } else if (hour == 12) {
      chime_idx_list[chime_len_idx++] = spk_time_cm_idx;
    } else {
      chime_idx_list[chime_len_idx++] = spk_time_pm_idx;
    }
  }

  // 播放 时
  if (is_12_hour) {
    // 12 小时 制
    int tmp_hour = hour - 12;
    if (tmp_hour == 0) {
      chime_idx_list[chime_len_idx++] = 10;
      chime_idx_list[chime_len_idx++] = 2;
    } else if (tmp_hour <= 10) {
      if (tmp_hour == 2) {
        chime_idx_list[chime_len_idx++] = tmp_hour + 10;  // 两 的 音频
      } else {
        chime_idx_list[chime_len_idx++] = tmp_hour;
      }
    } else {
      chime_idx_list[chime_len_idx++] = 10;
      chime_idx_list[chime_len_idx++] = tmp_hour - 10;
    }

  } else {

    // 24 小时 制
    if (hour < 10) {
      if (hour == 2) {
        chime_idx_list[chime_len_idx++] = hour + 10;  // 两 的 音频
      } else {
        chime_idx_list[chime_len_idx++] = hour;
      }
    } else if (hour >= 10 && hour < 20) {
      chime_idx_list[chime_len_idx++] = 10;
      if (hour - 10 > 0) {
        chime_idx_list[chime_len_idx++] = hour - 10;
      }
    } else if (hour >= 20) {
      chime_idx_list[chime_len_idx++] = 2;
      chime_idx_list[chime_len_idx++] = 10;
      if (hour - 20 > 0) {
        chime_idx_list[chime_len_idx++] = hour - 20;
      }
    } else {
    }
  }

  // 播放 点
  chime_idx_list[chime_len_idx++] = spk_time_dot_idx;

  //  播放 整
  if (minute == 0) {
    chime_idx_list[chime_len_idx++] = spk_time_whole_idx;
  } else {

    if (minute < 10) {
      chime_idx_list[chime_len_idx++] = 0;
    }

    int tmp_minute_div = minute / 10;
    int tmp_minute_mol = minute % 10;

    if (tmp_minute_div > 0) {
      chime_idx_list[chime_len_idx++] = tmp_minute_div;
    }

    if (minute >= 10) {
      chime_idx_list[chime_len_idx++] = 10;
    }

    if (tmp_minute_mol > 0) {
      chime_idx_list[chime_len_idx++] = tmp_minute_mol;
    }


    // 播放 分
    chime_idx_list[chime_len_idx++] = spk_time_minute_idx;
  }



  chime_len = chime_len_idx;

  chime_running = true;

  Serial_Loginfo("chime_idx_len: " + String(chime_len));
}

void clear_chime() {
  chime_len = 0;
  chime_crt = -1;
  chime_crt_done = true;
  chime_running = false;
}

bool chime_handler() {

  if (chime_handling) {
    return chime_running;
  }
  chime_handling = true;

  // 如果 chime 正在运行
  if (chime_running) {
    // 如果过当前任务 完成
    if (chime_crt_done) {
      chime_crt_done = false;
      // 首次
      if (chime_crt == -1) {
        chime_crt = 0;
      } else {
        chime_crt++;
      }
      if (chime_crt < chime_len) {
        start_chime(spk_time_list[chime_idx_list[chime_crt]].c_str());
      } else if (chime_crt == chime_len) {
        // 最后一次
        clear_chime();
        Serial_Loginfo("chime finished.");
      }
    } else {
      if (chime_mp3 != NULL && chime_mp3->isRunning()) {
        /* Commented out
        if (millis()-lastms > 1000) {
          lastms = millis();
          Serial.printf("Running for %d ms...\n", lastms);
          Serial.flush();
        }
        */
        if (!chime_mp3->loop()) {
          ready_next_chime();
          chime_crt_done = true;
        } else {
          chime_crt_done = false;
        }
      }

      // if (mp3 != NULL && mp3->isRunning()) {
      //   /* Commented out
      //   if (millis()-lastms > 1000) {
      //     lastms = millis();
      //     Serial.printf("Running for %d ms...\n", lastms);
      //     Serial.flush();
      //   }
      //   */
      //   if (!mp3->loop()) {
      //     ready_next_chime();
      //     chime_crt_done = true;
      //   }else{
      //     chime_crt_done = false;
      //   }
      // }
    }
  }

  chime_handling = false;

  return chime_running;
}

// =========================================
// define interrupt
// =========================================

// 当按下 停止 按钮时，执行此中断处理
void ICACHE_RAM_ATTR stop_btn_interrupt() {
  if (current_millis - stop_btn_lock < 500 && current_millis - stop_btn_lock > 0) return;
  stop_btn_lock = current_millis;
  Serial_Loginfo("触发 停止按钮 中断.");

  // 如果stop_btn 没有激活长按
  if(stop_btn_long_press == -1){
    stop_btn_long_press = 0;
  }


  // 检查是否在这个播放 音频如果有立刻打断

  if(mp3 != nullptr){
    mp3->desync();
    mp3_desync = true;
    audio_is_running = false;
  }


}

void stop_btn_long_press_10s(){
   Serial_Loginfo("触发 停止按钮 10s 长按.");

  // 移除 ap 账号密码
  remove_sys_value_to_spiffs("wifi_ap_name");
  remove_sys_value_to_spiffs("wifi_ap_passwd");

}

// 当 触发 6时 时，执行此中断处理
void ICACHE_RAM_ATTR hour_in_interrupt() {
  // 每次触发时，记录触发时的时间
  hour_in_enable = current_millis;
  if (current_millis - hour_in_lock < CLOCK_ZERO_POS_INTERVAL_MIN && current_millis - hour_in_lock > 0) return;
  hour_in_lock = current_millis;
  Serial_Loginfo("触发 6点 中断...");

  // 检查是否6点报告给主程序


  
}

// 当 触发 整点 时，执行此中断处理
void ICACHE_RAM_ATTR min_in_interrupt() {
  // 每次触发时，记录触发时的时间
  minute_in_enable = current_millis;
  if (current_millis - minute_in_lock < CLOCK_ZERO_POS_INTERVAL_MIN && current_millis - minute_in_lock > 0) return;
  minute_in_lock = current_millis;
  Serial_Loginfo("触发 整点 中断...");
}

bool is_hour_in_enable() {
  return hour_in_enable > 0
         && current_millis - hour_in_enable > 0
         && current_millis - hour_in_enable < 1500;
}

bool is_minute_in_enable() {
  return minute_in_enable > 0
         && current_millis - minute_in_enable > 0
         && current_millis - minute_in_enable < 1500;
}



void timer_1ms_handler() {

  // Serial_Loginfo("1ms");

  // Serial_Loginfo("1ms end");

  // 如果 6点 整
  // if(is_hour_in_enable() && is_minute_in_enable()){
  //   hollow_clock_zero_pos_notify();
  // }
}

void timer_2ms_handler() {

  // Serial_Loginfo("1ms");

  // 步进电机每1ms检查一次是否有异步任务需要处理
  stepper->rotateAsyncHandle();

  // Serial_Loginfo("1ms end");

  // 如果 6点 整
  if(is_hour_in_enable() && is_minute_in_enable()){
    hollow_clock_zero_pos_notify();
  }
}

// 时钟 0 定位 点 通知
void hollow_clock_zero_pos_notify(){

  // 当 到达 6点整 如果 现在在自动校时， 获取当前时间，和计算6点到 当前时间的偏差 进行

  Serial_Loginfo("hollow_clock_zero_pos_notify");


// 如果初始化过， 且 一小时内 再次 触发跳过
  if (proof_time_inited && current_millis - hour_in_lock < CLOCK_ZERO_POS_INTERVAL_MAX && current_millis - hour_in_lock > 0) {
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
  if (hours_range >= 0) {
    tmp_step = STEPS_PER_ROTATION * hours_range + ((STEPS_PER_ROTATION / 60) * minutes) + ((STEPS_PER_ROTATION / 60 / 60) * seconds);
_
    SerialLoginfo("6点 到 12点之间 正转 " + String(hours_range) + " " + String(minutes) + " " + String(seconds) + " steps:" + String(tmp_step));
    Serial_Loginfo("目标时间: " + String(hours) + ":" + String(minutes) + ":" + String(seconds));

  } else {

    hours_range = -hours_range;

    int pos_seconds = (60 - seconds) % 60;

    int pos_minutes = pos_seconds > 0
                        ? (60 - minutes - 1) % 60
                        : (60 - minutes) % 60;

    int pos_hours = pos_point_time - hours - 1;

    tmp_step = STEPS_PER_ROTATION * pos_hours + ((STEPS_PER_ROTATION / 60) * pos_minutes) + ((STEPS_PER_ROTATION / 60 / 60) * pos_seconds);

    tmp_step = -tmp_step;

    Serial_Loginfo("0点 到 6点之间 反转 " + String(pos_hours) + " " + String(pos_minutes) + " " + String(pos_seconds) + " steps:" + String(tmp_step));
    Serial_Loginfo("目标时间: " + String(hours) + ":" + String(minutes) + ":" + String(seconds));
  }

  stepper->rotateAsync(tmp_step, prood_time_after);

  

}



void timer_1000ms_handler() {
  int timer_val = 1000;
  // Serial_Loginfo("timer_1000ms_handler    1000ms");

  ntp_allow_update_flag = true;

  // 每秒 当前 时间戳 加 1
  current_timestamp++;

  // 检查 wifi 连接状态
  timer_check_wifi_connect_status();

  // 处理 时钟 定时器
  timer_clock_interval_handle(timer_val);

  // 处理 报时 时间检查
  chime_time_check_handler();

  // 处理 生日 时 唱 生日歌
  birthday_handler();

  // 处理 闹钟 任务
  if (bellManager != NULL) {
    bellManager->bell_handler();
  }

  // 如果stop 长按激活, 每秒增加1000
  // Serial_Loginfo(String("stop 长按激活 check\t") + String(stop_btn_long_press));
  if(stop_btn_long_press > -1){
    int stop_btn_status = digitalRead(STOP_BUTTON_PIN);

    Serial_Loginfo(String("stop_btn_status check\t") + stop_btn_status);
    if(stop_btn_status == LOW){
      stop_btn_long_press += 1000;
      if(stop_btn_long_press >= 10 * 1000){
        stop_btn_long_press_10s();
        stop_btn_long_press = -1;
      }
    }else{
      stop_btn_long_press = -1;
    }
  }
}

void timer_20000ms_handler() {

  Serial_Loginfo("20000ms");

}

// 五分钟 定时任务
void timer_300000ms_handler() {

  // stepper->rotateAsync(STEPS_PER_ROTATION/60);
  if (ntp_inited && WiFi.status() == WL_CONNECTED) {
    current_timestamp = network_timestamp;
  }
}

void timer_60000ms_handler() {
  int timer_val = 60000;

  timer_clock_interval_handle(timer_val);


  // 获取 当前 电池的电压，如果低于 3v说明快没电了
  float crt_voltage = get_battery_voltage();
  Serial_Loginfo(String("current voltage:") + crt_voltage + "V");

  // if(crt_voltage < 3){
  //   start_audio(spk_hungry.c_str());
  // }

}


// 处理 时钟每次移动的步数
void timer_clock_interval_handle(int timer_val) {

  if (clock_mode == CLOCK_MODE_SECOND && timer_val == 1000) {

    if (!stepper->isRunning()) {
      stepper->rotateAsync(0);
    }
    stepper->addRotateAsync(STEPS_PER_ROTATION / 60 / 60);
    int crt_second = get_seconds_from_current_timestamp();
    if (crt_second == 0) {
      // 补偿 每分钟缺少 32 步的问题
      // STEPS_PER_ROTATION / 60 / 60 = 8.533333333333333
      // 512 - (60 * 8 = 480) = 32
      stepper->addRotateAsync(32);
      // stepper->addRotateAsync((STEPS_PER_ROTATION / 60) - (60 * (STEPS_PER_ROTATION / 60 / 60)));
    }

  } else if (clock_mode == CLOCK_MODE_MINUTE && timer_val == 60000) {

    if (!stepper->isRunning()) {
      stepper->rotateAsync(0);
    }
    stepper->addRotateAsync(STEPS_PER_ROTATION / 60);
  }
}

void timer_check_wifi_connect_status() {

  //如果wifi连接成功，WiFi.status()返回值则为，WL_CONNECTED
  if (WiFi.status() == WL_CONNECTED && !wifi_connected_notify) {
    wifi_connected_notify = true;
    wifi_conneccted_notify_num++;

    wifi_connect_success_notify();
  }
}

// wifi 连接成功 通知 函数
void wifi_connect_success_notify() {

  Serial_Loginfo("Wi-Fi 连接成功");
  Serial_Loginfo("您的开发板的IP:");
  //当连接成功之后，向串口输出开发板的ip地址
  Serial_Loginfo(WiFi.localIP().toString());

  // 只有 第一次连接网络时 开始校时
  if (wifi_conneccted_notify_num == 1) {
    // 开始 校时
    start_proof_time();

    // 获取 网络时间
    get_network_time_after_wifi_connect();
  }

  // 初始化 OTA
  // setup_arduino_ota();

  start_audio(spk_wifi_connected.c_str());
}

void get_network_time_after_wifi_connect() {

  ntp_inited = false;
}

// 获取网络时间处理函数
void get_network_time_handler() {

  if (!ntp_inited) {
    timeClient.end();
    timeClient.begin();
    timeClient.setUpdateInterval(5 * 60 * 1000);  // 五分钟 更新同步一次网络时间
    timeClient.setTimeOffset(3600 * time_zone);   //+1区，偏移3600，+8区，偏移3600*8
  }

  if (!ntp_allow_update_flag || WiFi.status() != WL_CONNECTED) { return; }
  // 执行完后将标记置为false
  ntp_allow_update_flag = false;

  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();

  network_timestamp = epochTime;

  // 如果 ntp 首次初始化， 初始化完毕后， 设置网络时间为当前时间
  if (!ntp_inited) {
    ntp_inited = true;
    current_timestamp = network_timestamp;

    // 打印时间
    int currentHour = timeClient.getHours();
    int currentMinute = timeClient.getMinutes();
    int weekDay = timeClient.getDay();
    int currentSecond = timeClient.getSeconds();

    //将epochTime换算成年月日
    struct tm *ptm = gmtime((time_t *)&epochTime);

    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon + 1;
    int year = ptm->tm_year;

    Serial.printf("Epoch Time: %d current time: %d-%d-%d %d:%d:%d\n", epochTime, (year + 1900), currentMonth, monthDay, currentHour, currentMinute, currentSecond);
  }
}

int get_year_from_timestamp(time_t *timestamp) {
  struct tm *ptm = gmtime(timestamp);
  return ptm->tm_year + 1900;
}
int get_month_from_timestamp(time_t *timestamp) {
  struct tm *ptm = gmtime(timestamp);
  return ptm->tm_mon + 1;
}
int get_month_day_from_timestamp(time_t *timestamp) {
  struct tm *ptm = gmtime(timestamp);
  return ptm->tm_mday;
}
int get_hour_from_timestamp(time_t *timestamp) {
  struct tm *ptm = gmtime(timestamp);
  return ptm->tm_hour;
}
int get_minute_from_timestamp(time_t *timestamp) {
  struct tm *ptm = gmtime(timestamp);
  return ptm->tm_min;
}
int get_second_from_timestamp(time_t *timestamp) {
  struct tm *ptm = gmtime(timestamp);
  return ptm->tm_sec;
}
int get_week_from_timestamp(time_t *timestamp) {
  struct tm *ptm = gmtime(timestamp);
  return ptm->tm_wday;
}

String get_format_datetime_from_timestamp(time_t *timestamp) {
  int year = get_year_from_timestamp(timestamp);
  int month = get_month_from_timestamp(timestamp);
  int month_day = get_month_day_from_timestamp(timestamp);
  int hour = get_hour_from_timestamp(timestamp);
  int minutes = get_minute_from_timestamp(timestamp);
  int seconds = get_second_from_timestamp(timestamp);
  return String(year) + "-"
         + (month < 10 ? "0" : "") + String(month) + "-"
         + (month_day < 10 ? "0" : "") + String(month_day) + " "
         + (hour < 10 ? "0" : "") + String(hour) + ":"
         + (minutes < 10 ? "0" : "") + String(minutes) + ":"
         + (seconds < 10 ? "0" : "") + String(seconds);
}

int get_year_from_current_timestamp() {
  return get_year_from_timestamp((time_t *)&current_timestamp);
}
int get_month_from_current_timestamp() {
  return get_month_from_timestamp((time_t *)&current_timestamp);
}
int get_month_day_from_current_timestamp() {
  return get_month_day_from_timestamp((time_t *)&current_timestamp);
}

int get_hours_from_current_timestamp() {
  return get_hour_from_timestamp((time_t *)&current_timestamp);
}

int get_minutes_from_current_timestamp() {
  return get_minute_from_timestamp((time_t *)&current_timestamp);
}

int get_seconds_from_current_timestamp() {
  return get_second_from_timestamp((time_t *)&current_timestamp);
}



String get_format_datetime_from_current_timestamp() {
  return get_format_datetime_from_timestamp((time_t *)&current_timestamp);
}



// 初始化 串口
void setup_serial() {
  Serial_Loginfo("Init 串口");
  Serial.begin(SERIAL_BAUD_RATE);
}

// 初始化变量
void setup_init_variables() {

  // 设置为 当前时间确保可以在一开始触发事件
  stop_btn_lock = current_millis;
  hour_in_lock = current_millis;
  minute_in_lock = current_millis;


  // 是否正在播放音频 default: false
  audio_is_running = false;

  // 是否正在校对时间 default: false
  prood_time_is_runnning = false;

  // 6点触发中断 是否激活
  hour_in_enable = 0;
  minute_in_enable = 0;

  ntp_inited = true;
  // 时间允许获取 flag
  ntp_allow_update_flag = false;

  // wifi是否已连接
  wifi_connected_notify = false;
  wifi_conneccted_notify_num = 0;

  WiFi.disconnect(true, true);
  WiFi.enableSTA(true);
  WiFi.enableAP(true);
  WiFi.mode(WIFI_MODE_APSTA);


  Serial_Loginfo("Hollow Clock 4X");
  Serial_Loginfo(String("Version: ") + FIRMWARE_VERSION);
}

// 初始化 定时器
void setup_timer() {
  Serial_Loginfo("Init 定时器");

  ticker1ms.attach_ms(1, timer_1ms_handler);
  ticker1ms.attach_ms(2, timer_2ms_handler);
  ticker1000ms.attach_ms(1000, timer_1000ms_handler);
  ticker20000ms.attach_ms(20000, timer_20000ms_handler);
  ticker60000ms.attach_ms(60000, timer_60000ms_handler);
  ticker300000ms.attach_ms(300000, timer_300000ms_handler);

  Serial_Loginfo("Init 定时器 finish");
}



// 初始化 步进电机
void setup_stepper() {

  Serial_Loginfo("Init 步进电机");

  stepper = new Stepper(STEPPER_A_PIN, STEPPER_B_PIN, STEPPER_C_PIN, STEPPER_D_PIN);
}

// 初始化 停止按钮
void setup_stop_button() {
  Serial_Loginfo("Init 停止按钮");

  //设置中断触发程序
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(STOP_BUTTON_PIN, stop_btn_interrupt, FALLING);
}

// 初始化 霍尔传感器输入
void setup_hall_sensor() {
  Serial_Loginfo("Init 霍尔传感器输入");

  //设置中断触发程序
  pinMode(HOUR_IN_PIN, INPUT_PULLUP);
  attachInterrupt(HOUR_IN_PIN, hour_in_interrupt, FALLING);
  // attachInterrupt(HOUR_IN_PIN, hour_in_interrupt_after, RISING);

  pinMode(MINUTE_IN_PIN, INPUT_PULLUP);
  attachInterrupt(MINUTE_IN_PIN, min_in_interrupt, FALLING);
  // attachInterrupt(MINUTE_IN_PIN, min_in_interrupt_after, RISING);
}

// Called when a metadata event occurs (i.e. an ID3 tag, an ICY block, etc.
void MDCallback(void *cbData, const char *type, bool isUnicode, const char *string) {
  const char *ptr = reinterpret_cast<const char *>(cbData);
  (void)isUnicode;  // Punt this ball for now
  // Note that the type and string may be in PROGMEM, so copy them to RAM for printf
  char s1[32], s2[64];
  strncpy_P(s1, type, sizeof(s1));
  s1[sizeof(s1) - 1] = 0;
  strncpy_P(s2, string, sizeof(s2));
  s2[sizeof(s2) - 1] = 0;
  Serial.printf("METADATA(%s) '%s' = '%s'\n", ptr, s1, s2);
  Serial.flush();
}

// Called when there's a warning or error (like a buffer underflow or decode hiccup)
void StatusCallback(void *cbData, int code, const char *string) {
  const char *ptr = reinterpret_cast<const char *>(cbData);
  // Note that the string may be in PROGMEM, so copy it to RAM for printf
  char s1[64];
  strncpy_P(s1, string, sizeof(s1));
  s1[sizeof(s1) - 1] = 0;
  Serial.printf("STATUS(%s) '%d' = '%s'\n", ptr, code, s1);
  Serial.flush();
}

// 设置音量
void set_gain_audio(float gain) {
  if (out != NULL) {
    out->SetGain(gain);
  }
}

// 是否 播放 中
bool is_running_audio() {
  return mp3->isRunning();
}

// 开始播放音频
void start_audio(const char *url) {

  Serial_Loginfo("start_audio " + String(url));

  // stop_audio();

  AudioType type = Audio_SPIFFS_FILE;

  bool isHTTP = String(url).startsWith("http://");
  if (isHTTP) { type = Audio_HTTP_FILE; }

  // Serial_Loginfo("isHTTP: " + String(isHTTP));
  // Serial_Loginfo("AudioType: " + String(type));

  if (type == Audio_HTTP_FILE) {
    file = new AudioFileSourceICYStream(url);

    // 如果文件未找到
    if (!file->isOpen()) {
      Serial_Loginfo("Not Found Audio File");
      start_audio(spk_not_found_audio.c_str());
      delete file;
      file = nullptr;
      return;
    }

    buff = new AudioFileSourceBuffer(file, 4096);  // Doubled form default 2048

  } else if (type == Audio_SPIFFS_FILE) {
    spiffsfile = new AudioFileSourceSPIFFS(url);

    // 如果文件未找到
    if (!spiffsfile->isOpen()) {
      // 如果 找不到文件
      // 播放 未找到文件 音频
      Serial_Loginfo("Not Found Audio File");
      start_audio(spk_not_found_audio.c_str());
      delete spiffsfile;
      spiffsfile = nullptr;
      return;
    }

    buff = new AudioFileSourceBuffer(spiffsfile, 4096);  // Doubled form default 2048

  } else {
    // 如果 找不到文件
    // 播放 未知音频类型

    Serial_Loginfo("Unkown Audio Type");
    return;
  }

  // out = new AudioOutputI2S();
  // out->SetGain(i2s_gain);                         //设置音量
  // out->SetPinout(I2S_SCLK, I2S_LRC, I2S_DIN);     //设置接到MAX98357A的引脚, (串行时钟SCK)-->SCLK, (字选择WS)-->LRC, (串行数据SD)-->DIN
  mp3 = new AudioGeneratorMP3();

  mp3->begin(buff, out);

  audio_is_running = true;
}

// 停止播放音频
void pause_audio() {
  // no impl
}

// 停止播放音频
void stop_audio() {
  audio_is_running = false;

  if (mp3 != nullptr) {
    mp3->stop();
    delete mp3;
    mp3 = nullptr;
  }

  if (buff != nullptr) { 
    delete buff;  
    buff = nullptr;
  }
  // if(out != nullptr){ delete out; out = nullptr;  }
  if (file != nullptr) {
     delete file; 
     file = nullptr;
  }
  if (spiffsfile != nullptr) { 
    delete spiffsfile; 
    spiffsfile = nullptr;
 }

}

void audio_handler() {
  if (audio_is_running) {
    if (mp3 != nullptr && mp3->isRunning()) {
      /* Commented out
        if (millis()-lastms > 1000) {
          lastms = millis();
          Serial.printf("Running for %d ms...\n", lastms);
          Serial.flush();
        }
        */
      if (!mp3->loop()) {
        stop_audio();
      }
    }else{
      stop_audio();
    }
  }else{
    if(mp3_desync){
      stop_audio();
    }
  }
}


// 开始播放音频
void start_chime(const char *url) {

  Serial_Loginfo("start_chime " + String(url));

  // stop_audio();

  AudioType type = Audio_SPIFFS_FILE;

  bool isHTTP = String(url).startsWith("http://");
  if (isHTTP) { type = Audio_HTTP_FILE; }

  // Serial_Loginfo("isHTTP: " + String(isHTTP));
  // Serial_Loginfo("AudioType: " + String(type));

  if (type == Audio_HTTP_FILE) {
    chime_file = new AudioFileSourceICYStream(url);
    chime_buff = new AudioFileSourceBuffer(chime_file, 4096);  // Doubled form default 2048
  } else if (type == Audio_SPIFFS_FILE) {
    chime_spiffsfile = new AudioFileSourceSPIFFS(url);
    chime_buff = new AudioFileSourceBuffer(chime_spiffsfile, 4096);  // Doubled form default 2048

  } else {
    // 如果 找不到文件
    // 播放 未知音频类型

    Serial_Loginfo("Unkown Audio Type");
  }

  // out = new AudioOutputI2S();
  // out->SetGain(i2s_gain);                         //设置音量
  // out->SetPinout(I2S_SCLK, I2S_LRC, I2S_DIN);     //设置接到MAX98357A的引脚, (串行时钟SCK)-->SCLK, (字选择WS)-->LRC, (串行数据SD)-->DIN

  chime_mp3 = new AudioGeneratorMP3();

  chime_mp3->begin(chime_buff, out);
}

// 停止播放音频
void pause_chime() {
  // no impl
}

// 停止播放音频
void stop_chime() {
  chime_running = false;

  ready_next_chime();
}

// 停止播放音频
void ready_next_chime() {

  if (chime_mp3 != nullptr) {
    chime_mp3->stop();
    delete chime_mp3;
    chime_mp3 = nullptr;
  }
  if (chime_buff != nullptr) { 
    delete chime_buff; 
    chime_buff = nullptr; 
  }
  if (chime_file != nullptr) { 
    delete chime_file; 
    chime_file = nullptr;
  }
  if (chime_spiffsfile != nullptr) { 
    delete chime_spiffsfile; 
    chime_spiffsfile = nullptr;
  }
}

// void ready_next_chime(){

//   if(mp3 != NULL){
//     mp3->stop();
//     delete mp3;
//   }
//   if(buff != NULL){ delete buff; }
//   if(file != NULL){ delete file; }
//   if(spiffsfile != NULL){ delete spiffsfile; }
// }


// 初始化 音频输出
void setup_audio() {
  Serial_Loginfo("Init 音频输出");

  out = new AudioOutputI2S();
  out->SetGain(i2s_gain);                      //设置音量
  out->SetPinout(I2S_SCLK, I2S_LRC, I2S_DIN);  //设置接到MAX98357A的引脚, (串行时钟SCK)-->SCLK, (字选择WS)-->LRC, (串行数据SD)-->DIN
}



// 初始化 SPIFFS
void setup_spiffs() {
  Serial_Loginfo("Init SPIFFS");

  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial_Loginfo("[SPIFFS] mount failed");
  }

  Serial_Loginfo("[SPIFFS] totalBytes\t" + String(SPIFFS.totalBytes()));
  Serial_Loginfo("[SPIFFS] usedBytes\t\t" + String(SPIFFS.usedBytes()));
}

void setup_spiffs_config() {
  Serial_Loginfo("Init SPIFFS Config Info");

  String _wifi_ssid = get_sys_value_to_spiffs("wifi_ssid");
  String _wifi_pwd = get_sys_value_to_spiffs("wifi_pwd");
  String _wifi_ap_name = get_sys_value_to_spiffs("wifi_ap_name");
  String _wifi_ap_passwd = get_sys_value_to_spiffs("wifi_ap_passwd");
  String _clock_mode = get_sys_value_to_spiffs("clock_mode");
  String _clock_is_12_hour = get_sys_value_to_spiffs("clock_is_12_hour");
  String _time_zone = get_sys_value_to_spiffs("time_zone");
  String _birthday = get_sys_value_to_spiffs("birthday");
  String _gain = get_sys_value_to_spiffs("gain");


  Serial_Loginfo(String("[SPIFFS Config] wifi_ssid: ") + _wifi_ssid);
  Serial_Loginfo(String("[SPIFFS Config] wifi_pwd: ") + _wifi_pwd);
  Serial_Loginfo(String("[SPIFFS Config] wifi_ap_name: ") + _wifi_ap_name);
  Serial_Loginfo(String("[SPIFFS Config] wifi_ap_passwd: ") + _wifi_ap_passwd);
  Serial_Loginfo(String("[SPIFFS Config] clock_mode: ") + _clock_mode);
  Serial_Loginfo(String("[SPIFFS Config] clock_is_12_hour: ") + _clock_is_12_hour);
  Serial_Loginfo(String("[SPIFFS Config] time_zone: ") + _time_zone);
  Serial_Loginfo(String("[SPIFFS Config] birthday: ") + _birthday);
  Serial_Loginfo(String("[SPIFFS Config] gain: ") + _gain);

  if (_wifi_ap_name.length() > 0) {
    wifi_ap_name = _wifi_ap_name;
  }
  if (_wifi_ap_passwd.length() > 0) {
    wifi_ap_passwd = _wifi_ap_passwd;
  }

  if (_wifi_ssid.length() > 0) {
    wifi_ssid = _wifi_ssid;
  }
  if (_wifi_pwd.length() > 0) {
    wifi_pwd = _wifi_pwd;
  }

  if (_clock_mode.length() > 0) {
    clock_mode = _clock_mode.toInt();
  }

  if (_clock_is_12_hour.length() > 0) {
    clock_is_12_hour = _clock_is_12_hour.toInt();
  }

  if (_time_zone.length() > 0) {
    time_zone = _time_zone.toInt();
  }

  if (_birthday.length() > 0) {
    birthday = (unsigned long)atol(_birthday.c_str());
  }

  if (_gain.length() > 0) {
    i2s_gain = _gain.toFloat();
  }
}

void clear_sys_value_to_spiffs() {
  remove_sys_value_to_spiffs("wifi_ssid");
  remove_sys_value_to_spiffs("wifi_pwd");
  remove_sys_value_to_spiffs("wifi_ap_name");
  remove_sys_value_to_spiffs("wifi_ap_passwd");
  remove_sys_value_to_spiffs("clock_mode");
  remove_sys_value_to_spiffs("clock_is_12_hour");
  remove_sys_value_to_spiffs("time_zone");
  remove_sys_value_to_spiffs("birthday");
}

void set_sys_value_to_spiffs(String name, String val) {
  String prefix = "/sys/";
  File file = SPIFFS.open(prefix + name, "w");
  file.write((uint8_t *)val.c_str(), val.length());
  file.flush();
  file.close();
}

String get_sys_value_to_spiffs(String name) {
  String prefix = "/sys/";
  if (!SPIFFS.exists(prefix + name)) {
    return "";
  }
  File file = SPIFFS.open(prefix + name, "r");
  String str = file.readString();
  file.close();
  return str;
}

bool remove_sys_value_to_spiffs(String name) {
  String prefix = "/sys/";
  if (SPIFFS.exists(prefix + name)) {
    SPIFFS.remove(prefix + name);
    return true;
  } else {
    return false;
  }
}

// 初始化 wifi ap
void setup_wifi_ap() {
  Serial_Loginfo("Init Wi-Fi AP");

  // 设置内网
  IPAddress softLocal(WIFI_AP_IP);  // 1 设置内网WIFI IP地址
  IPAddress softGateway(WIFI_AP_IP);
  IPAddress softSubnet(255, 255, 255, 0);
  WiFi.softAPConfig(softLocal, softGateway, softSubnet);

  // 2 设置WIFI名称
  const char *softAPName = wifi_ap_name.c_str();

  // 3创建wifi  名称 +密码 如果没有密码不设置
  if (wifi_ap_passwd.length() > 0) {
    WiFi.softAP(softAPName, wifi_ap_passwd.c_str());
  } else {
    WiFi.softAP(softAPName);
  }

  // 4输出创建的WIFI IP地址
  IPAddress myIP = WiFi.softAPIP();

  // 5输出WIFI 名称
  Serial_Loginfo("[Wi-Fi] softAPName: ");
  Serial_Loginfo(softAPName);
  Serial_Loginfo("[Wi-Fi] AP IP address: ");
  Serial_Loginfo(myIP.toString());

  Serial_Loginfo("Init Wi-Fi AP Finish");
}


// 初始化 dns server
void setup_dnsserver() {
  Serial_Loginfo("Init DNS Server");

  /* Setup the DNS server redirecting all the domains to the apIP */
  dns_server.setErrorReplyCode(DNSReplyCode::NoError);
  dns_server.start(DNS_PORT, "*", WiFi.softAPIP());
}


// 首页处理
void httpd_handler_index() {

  Serial_Logdebug("[httpd] /index.html 200");

  String path = "/index.html";
  String contentType = getContentType(path);     // 获取文件类型
  if (SPIFFS.exists(path)) {                     // 如果访问的文件可以在SPIFFS中找到
    File file = SPIFFS.open(path, "r");          // 则尝试打开该文件
    httpd_server.streamFile(file, contentType);  // 并且将该文件返回给浏览器
    file.close();                                // 并且关闭文件
    return;                                      // 返回true
  } else {
    httpd_server.send(200, "text/html", "<h1>Hollow Clock 4X</h1> <p>未初始化</p>");
  }
}

void httpd_handler_init_html() {

  String path = "/init.html";
  String contentType = getContentType(path);     // 获取文件类型
  if (SPIFFS.exists(path)) {                     // 如果访问的文件可以在SPIFFS中找到
    File file = SPIFFS.open(path, "r");          // 则尝试打开该文件
    httpd_server.streamFile(file, contentType);  // 并且将该文件返回给浏览器
    file.close();                                // 并且关闭文件
    return;                                      // 返回true
  }

  httpd_server.send(200,
                    "text/html",
                    "<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\"><title>Clock Init</title><meta charset=\"utf-8\"><meta http-equiv=\"X-UA-Compatible\"content=\"IE=edge\"><meta name=\"viewport\"content=\"width=device-width, initial-scale=1\"></head><body><textarea class=\"init_text_input\"style=\"width: 100%;height: 200px;\"value=\"\"></textarea><div class=\"submit\">提交</div><br/><br/><br/><textarea class=\"delete_spiffs_text_input\"style=\"width: 100%;height: 200px;\"value=\"\"></textarea><div class=\"delete_spiffs_submit\">删除SPIFFS内容</div><br/><br/><br/><div class=\"clear_sys_val\">清理系统变量</div><script type=\"application/javascript\">document.onreadystatechange=function(){document.querySelector(\".submit\").onclick=async function(){var val=document.querySelector(\".init_text_input\").value;var val_items=val.trim().split(\"\n\");for(var i=0;i<val_items.length;i++){var val_item=val_items[i].trim();if(val_item.length==0){continue}var val_arr=val_item.split(\" \");var val_path=val_arr[0].trim();var val_url=val_arr[1].trim();var resp=await fetch('./api/init_file_upload?path='+val_path+'&url='+val_url,{method:'get',headers:{'Content-Type':'text/plain'}}).then((res)=>res.text());console.log(resp)}};document.querySelector(\".delete_spiffs_submit\").onclick=async function(){var val=document.querySelector(\".delete_spiffs_text_input\").value;var resp=await fetch('./api/delete_spiffs_path?path='+val,{method:'get',headers:{'Content-Type':'text/plain'}}).then((res)=>res.text());alert(resp)};document.querySelector(\".clear_sys_val\").onclick=async function(){var resp=await fetch('./api/clear_sys_val',{method:'get',headers:{'Content-Type':'text/plain'}}).then((res)=>res.text());alert(resp)}}</script></body></html>");
}

// 初始化
void httpd_handler_init_file_upload() {
  String path = httpd_server.arg("path");
  String url = httpd_server.arg("url");
  Serial_Loginfo(path + " " + url);

  WiFiClient wifiClient;
  HTTPClient httpClient;

  //重点2 通过begin函数配置请求地址。此处也可以不使用端口号和PATH而单纯的
  httpClient.begin(wifiClient, url);
  Serial_Logdebug("URL: " + url);


  //重点3 通过GET函数启动连接并发送HTTP请求
  int httpCode = httpClient.GET();
  Serial_Logdebug("Send GET request to URL: " + url);

  String r = "";

  //重点4. 如果服务器响应HTTP_CODE_OK(200)则从服务器获取响应体信息并通过串口输出
  //如果服务器不响应HTTP_CODE_OK(200)则将服务器响应状态码通过串口输出
  if (httpCode == HTTP_CODE_OK) {
    // 使用getString函数获取服务器响应体内容
    WiFiClient &stream = httpClient.getStream();

    if (stream.available()) {
      Serial_Logdebug("stream.available()");

      // 打开文件
      File file = SPIFFS.open(path, "w");

      int stream_size = httpClient.getSize();

      Serial_Logdebug("file availableForWrite " + String(file.availableForWrite()));
      Serial_Logdebug("transfer Total Bytes: " + String(stream_size));

      int buf_size = 1024;
      uint8_t buf[buf_size];
      int len = -1;
      while ((len = stream.readBytes(buf, buf_size)) > 0) {
        file.write(buf, len);
      }
      file.flush();
      // 关闭文件
      file.close();

      Serial_Logdebug("file exists check: " + String(SPIFFS.exists(path)));

      r = "OK";
    } else {
      r = "Stream Unavailable";
    }
  } else {
    r = "FAILED " + String(httpCode);
  }

  //重点5. 关闭ESP8266与服务器连接
  httpClient.end();
  wifiClient.stop();

  String resp_str = path + " " + r;
  Serial_Logdebug(resp_str);
  httpd_server.send(200, "text/plain", resp_str);
}

void httpd_handler_device_info() {

  Serial_Logdebug("[httpd] /api/device_info 200");

  httpd_set_header_cros();

  httpd_server.send(
    200,
    "application/json;charset=utf-8",
    String("{")
      + "\"version\":" + String(FIRMWARE_VERSION) + ","
      + "\"time\":\"" + get_format_datetime_from_current_timestamp() + "\","
      + "\"crt_wifi_ssid\":\"" + wifi_ssid.c_str() + "\","
      + "\"crt_wifi_pwd\":\"" + wifi_pwd.c_str() + "\","
      + "\"crt_wifi_status\":" + String(WiFi.status()) + ","
      + "\"crt_ap_ssid\":\"" + wifi_ap_name.c_str() + "\","
      + "\"crt_ap_pwd\":\"" + wifi_ap_passwd.c_str() + "\","
      + "\"clock_mode\":" + String(clock_mode) + ","
      + "\"is_12_hour\":" + String(clock_is_12_hour ? "true" : "false") + ","
      + "\"time_zone\":" + time_zone + ","
      + "\"fs_total\":" + String(SPIFFS.totalBytes()) + ","
      + "\"fs_used\":" + String(SPIFFS.usedBytes()) + ","
      + "\"gain\":" + String(i2s_gain) + ","
      + "\"birthday\":" + String(birthday)
      + "}");
}

void httpd_handler_get_wifi_list() {

  httpd_set_header_cros();


  int n = WiFi.scanNetworks();

  String r = "{";
  r += "\"list\":[";
  for (int i = 0; i < n; i++) {
    r += "{\"name\":\"" + WiFi.SSID(i) + "\", \"checked\":" + (wifi_ssid == WiFi.SSID(i) ? "true" : "false") + "}" + ((i < n - 1) ? "," : "");
  }
  r += "],";
  r += "\"total\":" + String(n);
  r += "}";

  httpd_server.send(
    200,
    "application/json;charset=utf-8",
    r);
}

void httpd_handler_use_browser_time() {

  httpd_set_header_cros();

  String browser_time = httpd_server.arg("browser_time");
  String network_time = httpd_server.arg("network_time");
  String ret = "OK";
  if (browser_time.length() > 0) {
    long browser_time_long = atol(browser_time.c_str());
    unsigned long browser_timestamp = browser_time_long < 0 ? 0 : browser_time_long;
    current_timestamp = browser_timestamp;
    start_audio(spk_prood_time.c_str());
    start_proof_time();
  } else {
    if (WiFi.status() == WL_CONNECTED) {
      current_timestamp = network_timestamp;
      start_audio(spk_prood_time.c_str());
      start_proof_time();
    } else {
      start_audio(spk_not_net.c_str());
      ret = "FAILED";
    }
  }

  httpd_server.send(
    200,
    "text/plain",
    ret);
}

void httpd_handler_change_wifi() {

  httpd_set_header_cros();

  String ssid = httpd_server.arg("ssid");
  String pwd = httpd_server.arg("pwd");

  // WiFi.disconnect(true, false);

  Serial_Loginfo("ssid: " + ssid + "\tpwd: " + pwd);

  wifi_ssid = ssid;
  wifi_pwd = pwd;

  // TODO 保存 wifi信息

  set_sys_value_to_spiffs("wifi_ssid", wifi_ssid);
  set_sys_value_to_spiffs("wifi_pwd", wifi_pwd);


  httpd_server.send(
    200,
    "text/plain",
    "OK");

  WiFi.begin(wifi_ssid.c_str(), wifi_pwd.c_str());

  wifi_connected_notify = false;
}

void httpd_handler_get_wifi_status() {
  httpd_set_header_cros();

  httpd_server.send(
    200,
    "text/plain",
    String(WiFi.status()));
}

void httpd_handler_save_config() {

  httpd_set_header_cros();

  String _wifi_ap_name = httpd_server.arg("wifi_ap_name");
  String _wifi_ap_passwd = httpd_server.arg("wifi_ap_passwd");
  String _clock_mode = httpd_server.arg("clock_mode");
  String _clock_is_12_hour = httpd_server.arg("clock_is_12_hour");
  String _time_zone = httpd_server.arg("time_zone");
  String _birthday = httpd_server.arg("birthday");

  String restart = httpd_server.arg("restart");

  if (_wifi_ap_name.length() > 0 && _wifi_ap_name != wifi_ap_name) {
    wifi_ap_name = _wifi_ap_name;
    set_sys_value_to_spiffs("wifi_ap_name", wifi_ap_name);
  }

  if (_wifi_ap_passwd.length() > 0 && _wifi_ap_passwd != wifi_ap_passwd) {
    wifi_ap_passwd = _wifi_ap_passwd;
    set_sys_value_to_spiffs("wifi_ap_passwd", wifi_ap_passwd);
  }

  if (_clock_mode.length() > 0 && _clock_mode.toInt() != clock_mode) {
    clock_mode = _clock_mode.toInt();
    set_sys_value_to_spiffs("clock_mode", _clock_mode);
  }

  if (_clock_is_12_hour.length() > 0 && _clock_is_12_hour.toInt() != clock_is_12_hour) {
    clock_is_12_hour = _clock_is_12_hour.toInt();
    set_sys_value_to_spiffs("clock_is_12_hour", _clock_is_12_hour);
  }

  if (_time_zone.length() > 0 && _time_zone.toInt() != time_zone) {
    time_zone = _time_zone.toInt();
    set_sys_value_to_spiffs("time_zone", _time_zone);
  }

  if (_birthday.length() > 0) {
    birthday = (unsigned long)atol(_birthday.c_str());
    set_sys_value_to_spiffs("birthday", _birthday);
  }

  httpd_server.send(
    200,
    "text/plain",
    "OK");

  if (restart == "true") {
    delay(3000);
    reboot();
  }
}

void httpd_handler_clear_sys_val() {
  httpd_set_header_cros();

  clear_sys_value_to_spiffs();
  httpd_server.send(
    200,
    "text/plain",
    "OK");
}

void httpd_handler_format_spiffs(){
   httpd_set_header_cros();

  SPIFFS.format();
  
  httpd_server.send(
    200,
    "text/plain",
    "OK");
}

void httpd_handler_spiffs_content(){
  httpd_set_header_cros();

  String path = httpd_server.arg("path");
  
  String ret = "OK";

  String cont = "";
  if(!SPIFFS.exists(path)){
    ret = "Not Exist";
  }else{
    File file = SPIFFS.open(path, "r");
    cont = file.readString();
    file.close();
    ret += "\n";
    ret += cont;
  }
  
  httpd_server.send(
    200,
    "text/plain",
    ret);
}

void httpd_handler_delete_spiffs_path() {
  httpd_set_header_cros();

  String ret = "OK";

  String path = httpd_server.arg("path");

  if (SPIFFS.exists(path)) {
    SPIFFS.remove(path);
  } else {
    ret = "FAILED: NOT EXISTS";
  }

  httpd_server.send(
    200,
    "text/plain",
    ret);
}

void httpd_handler_check_firmware_upgrade() {

  httpd_set_header_cros();

  WiFiClient wifiClient;
  HTTPClient httpClient;

  String url = "http://mithub.oss-cn-beijing.aliyuncs.com/data/hollow_clock/firmware/upgrade.json";

  //重点2 通过begin函数配置请求地址。此处也可以不使用端口号和PATH而单纯的
  httpClient.begin(wifiClient, url);
  Serial_Logdebug("URL: " + url);


  //重点3 通过GET函数启动连接并发送HTTP请求
  int httpCode = httpClient.GET();
  Serial_Logdebug("Send GET request to URL: " + url);

  String r = "";
  String content = "";

  //重点4. 如果服务器响应HTTP_CODE_OK(200)则从服务器获取响应体信息并通过串口输出
  //如果服务器不响应HTTP_CODE_OK(200)则将服务器响应状态码通过串口输出
  if (httpCode == HTTP_CODE_OK) {
    // 使用getString函数获取服务器响应体内容
    WiFiClient &stream = httpClient.getStream();
    if (stream.available()) {
      Serial_Logdebug("stream.available()");
      content = stream.readString();
      Serial_Logdebug(String("content: ") + content);
    } else {
      r = "Stream Unavailable";
    }
  } else {
    r = "FAILED " + String(httpCode);
  }

  if (content.length() > 0) {
    httpd_server.send(
      200,
      "application/json;charset=utf-8",
      content);
  } else {
    httpd_server.send(
      200,
      "application/json;charset=utf-8",
      String("{")
        + "\"code\": 1, "
        + "\"msg\": \"" + r + "\""
        + "}");
  }

  //重点5. 关闭ESP8266与服务器连接
  httpClient.end();
  wifiClient.stop();

  // http_server_client.stop();
}

void httpd_handler_firmware_upgrade() {

  httpd_set_header_cros();

  String binfile = httpd_server.arg("binfile");

  String ret = firmware_upgrade(binfile);

  httpd_server.send(
    200,
    "application/json;charset=utf-8",
    ret);
}

void httpd_handler_set_gain() {
  String gain = httpd_server.arg("gain");
  if (gain.length() > 0) {
    float gain_f = gain.toFloat();
    i2s_gain = gain_f;
    set_gain_audio(gain_f);
    set_sys_value_to_spiffs("gain", String(gain));
  }

  httpd_server.send(
    200,
    "application/json;charset=utf-8",
    String(i2s_gain));
}

void httpd_handler_save_alarm() {

  if(httpd_server.method() == HTTP_OPTIONS){ 
    httpd_server.send(200, "text/plain", "");
    return; 
  }

  String title = httpd_server.arg("title");
  String hour = httpd_server.arg("hour");
  String minute = httpd_server.arg("minute");
  String week = httpd_server.arg("report_week");
  String report = httpd_server.arg("report");
  String audio = httpd_server.arg("audio");
  String enable_str = httpd_server.arg("enable");

  String cron = minute + " " + hour + " * * " + week;

  Serial_Loginfo(String("week :") + week);
  Serial_Loginfo(String("save alarm :" + title + "\t" + cron + "\t" + report + "\t" + audio + "\t" + enable_str));

  bool enable = enable_str.toInt();

  bool exists = bellManager->existBell(title.c_str());

  Serial_Loginfo(String("exist alarm :") + exists);

  BellItem *bellItem;
  if (exists) {
    bellItem = bellManager->getBell(title.c_str());
  } else {
    bellItem = new BellItem();
    bellItem->set_title(title.c_str());
    bellItem->set_enable(true);
  }

  bellItem->set_cron(cron.c_str());
  bellItem->set_report(report.toInt());
  bellItem->set_audio(audio.c_str());
  // bellItem->set_enable(enable);
  bellItem->set_once_finished(false);

  bool ret = true;

  if (!exists) {
    ret = bellManager->addBell(bellItem);
  } else {
    ret = bellManager->updateBell(bellItem);
  }

  httpd_server.send(
    200,
    "application/json;charset=utf-8",
    String(ret ? "OK" : "FAILED"));
}

void httpd_handler_delete_alarm() {
  if(httpd_server.method() == HTTP_OPTIONS){ 
    httpd_server.send(200, "text/plain", "");
    return; 
  }

  String title = httpd_server.arg("title");
  bool ret = bellManager->removeBell(title.c_str());

  httpd_server.send(
    200,
    "application/json;charset=utf-8",
    String(ret ? "OK" : "FAILED"));
}

void httpd_handler_alarm_list() {

  String ret = "{";
  ret += "\"code\":0,";
  ret += "\"msg\":\"SUCCESS\",";
  ret += "\"list\": [";

  for (int i = 0; i < bellManager->get_total(); i++) {
    BellItem *item = bellManager->getByIndex(i);

    // Serial_Loginfo(String("alarm list: ") + item->to_string().c_str());
    
    ret += String("{\"title\":\"") + item->get_title()
           + "\", \"time\":\"" + (item->get_hour() < 10 ? "0" : "") + item->get_hour() + ":" + (item->get_minute() < 10 ? "0" : "") + item->get_minute() + "\","
           + "\"report\":" + item->get_report() + ","
           + "\"week\":" + item->get_week() + ","
           + "\"enable\":" + (item->get_enable() ? "true" : "false") + ","
           + "\"audio\":\"" + item->get_audio() + "\""
           + "}";

    if (i < (bellManager->get_total() - 1)) {
      ret += ",";
    }
  }


  ret += "]";
  ret += "}";

  httpd_server.send(
    200,
    "application/json;charset=utf-8",
    ret);
}

void httpd_handler_alarm_info() {

  String title = httpd_server.arg("title");

  String ret = "{";
  ret += "\"code\":0,";
  ret += "\"msg\":\"SUCCESS\",";
  ret += "\"data\": ";

  BellItem *item = bellManager->getBell(title.c_str());
  ret += String("{\"title\":\"") + item->get_title()
         + "\", \"time\":\"" + (item->get_hour() < 10 ? "0" : "") + item->get_hour() + ":" + (item->get_minute() < 10 ? "0" : "") + item->get_minute() + "\","
         + "\"report\":" + item->get_report() + ","
         + "\"week\":" + item->get_week() + ","
         + "\"enable\":" + (item->get_enable() ? "true" : "false") + ","
         + "\"audio\":\"" + item->get_audio() + "\""
         + "}";

  ret += "}";

  httpd_server.send(
    200,
    "application/json;charset=utf-8",
    ret);
}

void httpd_handler_toggle_enable(){
  String title = httpd_server.arg("title");
  String enable_str = httpd_server.arg("enable");
  bool enable = enable_str == "true" ? true : false;

  BellItem *item = bellManager->getBell(title.c_str());
  item->set_enable(enable);

  bellManager->updateBell(item, true);

  httpd_server.send(
    200,
    "application/json;charset=utf-8",
    "OK"
  );

}

File fsUploadFile;
String fsUploadFileName;
String fsUploadFilePath;


void httpd_handler_add_audio(){
  String title = httpd_server.arg("title");
  String path = httpd_server.arg("path");

  Serial.println("File title: " + title); // 通过串口监视器输出上传文件的名称
  Serial.println("File path: " + path); // 通过串口监视器输出上传文件的名称

  if(path.length() > 0){

    fsUploadFilePath = path;
    write_bell_audio_to_memory(title, fsUploadFilePath);
    write_bell_audio_to_spiffs();

    httpd_server.send(
      200,
      "text/html;charset=utf-8",
      "<h1>上传成功</h1><script>setTimeout(function(){ history.back(); },1000);</script>"
    );

  }else{
    HTTPUpload &upload = httpd_server.upload();
  
    if (upload.status == UPLOAD_FILE_START) { // 如果上传状态为UPLOAD_FILE_START
      Serial.println("upload start."); 

      String filename = upload.filename; // 建立字符串变量用于存放上传文件名
      // if (!filename.startsWith("/"))
      //     filename = "/" + filename;            // 为上传文件名前加上"/"
      Serial.println("File Name: " + filename); // 通过串口监视器输出上传文件的名称

      fsUploadFileName = filename;

      int bell_audio_idx = generate_bell_audio_idx(fsUploadFileName);

      fsUploadFilePath = String("/audio/") + bell_audio_idx + ".mp3";

      fsUploadFile = SPIFFS.open(fsUploadFilePath, "w"); // 在SPIFFS中建立文件用于写入用户上传的文件数据
    } else if (upload.status == UPLOAD_FILE_WRITE) { // 如果上传状态为UPLOAD_FILE_WRITE
      Serial.println("upload write."); 
      if (fsUploadFile){
          fsUploadFile.write(upload.buf, upload.currentSize); // 向SPIFFS文件写入浏览器发来的文件数据
      }
    } else if (upload.status == UPLOAD_FILE_END) { // 如果上传状态为UPLOAD_FILE_END
      Serial.println("upload end."); 
      if (fsUploadFile) {                                                           // 如果文件成功建立
          fsUploadFile.close();                                   // 将文件关闭
          Serial.println(" Size: " + upload.totalSize);           // 通过串口监视器输出文件大小
          httpd_server.send(
            200,
            "text/html;charset=utf-8",
            "<h1>上传成功</h1><script>setTimeout(function(){ history.back(); },1000);</script>"
          );

          write_bell_audio_to_memory(fsUploadFileName, fsUploadFilePath);
          write_bell_audio_to_spiffs();
          
      } else {                                                                        // 如果文件未能成功建立
          Serial.println("File upload failed");                                // 通过串口监视器输出报错信息
          httpd_server.send(
            200,
            "text/html;charset=utf-8",
            "<h1>500: couldn't create file</h1>"
          );
      }
    }    
  }

  

}

void httpd_handler_remove_audio(){
  String title = httpd_server.arg("title");

  String audio_path = "";
  int idx = -1;

  for(int i = 0; i < bells_audio_list_len; i++){
    if(bells_audio_list[i][0] == title){
      audio_path = bells_audio_list[i][1];
      idx = i;
      break;
    }
  }

  if(audio_path.length() > 0){
    bool ishttp = audio_path.startsWith("http://");

    if(!ishttp){
      SPIFFS.remove(audio_path);
    }

    if(idx > -1){
      for(int i = idx; i < bells_audio_list_len; i++){
        if(i == bells_audio_list_len - 1){
          bells_audio_list[i][0] = "";
          bells_audio_list[i][1] = "";
        }else{
          bells_audio_list[i][0] = bells_audio_list[i+1][0];
          bells_audio_list[i][1] = bells_audio_list[i+1][1];
        } 
      }

      bells_audio_list_len--;

    }

    write_bell_audio_to_spiffs();

  }
  
  httpd_server.send(
    200,
    "text/plain;charset=utf-8",
    "OK"
  );
}

int generate_bell_audio_idx(String title) {
  String key = "gen_audio_idx";
  String idx_str = get_sys_value_to_spiffs(key);
  if(idx_str.length() > 0){
    int val = idx_str.toInt()+1;
    set_sys_value_to_spiffs(key, String(val));    
    return val;
  }else{
    int val = 10;
    set_sys_value_to_spiffs(key, String(val));    
    return val;
  }
}

void write_bell_audio_to_memory(String title, String path) {
  int idx = -1;

  String audio_path = String(path);

  for(int i = 0; i < bells_audio_list_len; i++){
    if(bells_audio_list[i][0] == title){
      idx = i;
      break;
    }
  }

  bool ishttps = audio_path.startsWith("https://");

  if(ishttps){
    audio_path.replace("https://", "http://");
  }

  // 已经存在
  if(idx > -1){
    bells_audio_list[idx][1] = audio_path;
  }else{
    bells_audio_list[bells_audio_list_len][0] = title;
    bells_audio_list[bells_audio_list_len][1] = audio_path;
    bells_audio_list_len++;
  }
}

void write_bell_audio_to_spiffs() {
  File file = SPIFFS.open(bells_audio_list_info, "w");   

    for(int i = 0; i < bells_audio_list_len; i++){
      String idx_str = String(i);
      file.write((uint8_t *)idx_str.c_str(), idx_str.length());
      file.write((uint8_t *)"\t", 1);
      file.write((uint8_t *)bells_audio_list[i][0].c_str(), bells_audio_list[i][0].length());
      file.write((uint8_t *)"\t", 1);
      file.write((uint8_t *)bells_audio_list[i][1].c_str(), bells_audio_list[i][1].length());
      if(i < bells_audio_list_len - 1){
        file.write((uint8_t *)"\n", 1);        
      }
    }
    file.flush();
    
    file.close();
}

void load_audio_config_to_memory(){

  if (SPIFFS.exists(bells_audio_list_info)) {

    File bells_audio_list_info_file = SPIFFS.open(bells_audio_list_info, "r");

    String cont = bells_audio_list_info_file.readString();

    Serial_Loginfo("load_bells_audio_config_from_spiffs list.txt content:");
    Serial_Loginfo(cont.c_str());

    int bells_titles_num = 0;

    int last_idx = 0;
    int str_idx = 0;

    if(cont.indexOf("\n", last_idx) == -1){
      if(cont.length() > 0){
        load_bells_audio_item_from_spiffs(cont.c_str());
      }
    }else{
      while ((str_idx = cont.indexOf("\n", last_idx)) > -1) {
        String title_item = cont.substring(last_idx, str_idx);

        Serial_Loginfo(String("title_item") + title_item);

        load_bells_audio_item_from_spiffs(title_item.c_str());

        last_idx = str_idx + 1;
        bells_titles_num++;
      }

      Serial_Loginfo(String("last_idx ") + last_idx);
      String title_item = cont.substring(last_idx, cont.length());
      Serial_Loginfo(String("title_item ") + title_item);
      load_bells_audio_item_from_spiffs(title_item.c_str());

    }
    bells_audio_list_info_file.close();
  }else{
    Serial_Loginfo("load_bells_config_from_spiffs list.txt not exists");
  }

}

void load_bells_audio_item_from_spiffs(String bell_audio_item){

    String cont = bell_audio_item;
    int last_idx = 0;
    int str_idx = 0;
    int bells_info_num = 0;

    int idx = -1;
    String title = "";

    while ((str_idx = cont.indexOf("\t", last_idx)) > -1) {
      String cont_item = cont.substring(last_idx, str_idx);

      if (bells_info_num == 0) {
        idx = cont_item.toInt();
      }
      if (bells_info_num == 1) {
        title = String(cont_item);
      }
      
      last_idx = str_idx + 1;
      bells_info_num++;
    }

    String bells_audio_path_str = cont.substring(last_idx, cont.length());


  if (title.length() > 0 && bells_audio_path_str.length() > 0) {
    bells_audio_list[idx][0] = title;
    bells_audio_list[idx][1] = bells_audio_path_str;

    bells_audio_list_len++;

    Serial_Loginfo(String("load_bells_audio_item_from_spiffs add bell: ") + title);
  }
 
}

void httpd_handler_audio_list(){

  if(!SPIFFS.exists(bells_audio_list_info)){
    httpd_server.send(
      200,
      "text/plain;charset=utf-8",
      ""
    );
    return;
  }

  File file = SPIFFS.open(bells_audio_list_info, "r");
  String cont = file.readString();
  httpd_server.send(
    200,
    "text/plain;charset=utf-8",
    cont
  );
  file.close();
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

bool httpd_handler_file_read(String path) {  // 处理浏览器HTTP访问

  if (path.endsWith("/")) {  // 如果访问地址以"/"为结尾
    path = "/index.html";    // 则将访问地址修改为/index.html便于SPIFFS访问
  }

  String contentType = getContentType(path);  // 获取文件类型

  if (SPIFFS.exists(path)) {                     // 如果访问的文件可以在SPIFFS中找到
    File file = SPIFFS.open(path, "r");          // 则尝试打开该文件
    httpd_server.streamFile(file, contentType);  // 并且将该文件返回给浏览器
    file.close();                                // 并且关闭文件
    return true;                                 // 返回true
  }
  return false;  // 如果文件未找到，则返回false
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
  if (!isIp(httpd_server.hostHeader())) {
    Serial_Loginfo("Request redirected to captive portal");
    // httpd_server.sendHeader("Location", String("http://") + toStringIp(httpd_server.client().localIP()), true);

    httpd_server.sendHeader("Location", String("http://") + WIFI_IP_DOMAIN, true);

    httpd_server.send(302, "text/plain", "");  // Empty content inhibits Content-length header so we have to close the socket ourselves.
    httpd_server.client().stop();              // Stop is needed because we sent no content length
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
  else if (filename.endsWith(".svg"))
    return "image/svg+xml";
  return "text/plain";
}

// 初始化 web server
void setup_httpd() {
  Serial_Loginfo("Init WebServer");

  httpd_server.on("/", httpd_handler_index);  // Index

  httpd_server.on("/init.html", httpd_handler_init_html);  // Index

  // 初始化 文件
  httpd_server.on("/api/init_file_upload", httpd_handler_init_file_upload);  // Index

  // 获取 设备信息
  httpd_server.on("/api/device_info", httpd_handler_device_info);  // Index

  // 获取wifi列表
  httpd_server.on("/api/get_wifi_list", httpd_handler_get_wifi_list);  // Index

  // 使用浏览器时间
  httpd_server.on("/api/use_browser_time", httpd_handler_use_browser_time);  // Index

  // 变更 wifi
  httpd_server.on("/api/change_wifi", httpd_handler_change_wifi);  // Index

  // 获取wifi 状态
  httpd_server.on("/api/get_wifi_status", httpd_handler_get_wifi_status);  // Index

  // 保存配置
  httpd_server.on("/api/save_config", httpd_handler_save_config);  // Index

  // 保存配置
  httpd_server.on("/api/clear_sys_val", httpd_handler_clear_sys_val);  // Index

  httpd_server.on("/api/format_spiffs", httpd_handler_format_spiffs);  // Index

  httpd_server.on("/api/get_spiffs_content", httpd_handler_spiffs_content);  // Index

  httpd_server.on("/api/delete_spiffs_path", httpd_handler_delete_spiffs_path);  // Index

  httpd_server.on("/api/check_firmware_upgrade", httpd_handler_check_firmware_upgrade);

  httpd_server.on("/api/firmware_upgrade", httpd_handler_firmware_upgrade);

  httpd_server.on("/api/set_gain", httpd_handler_set_gain);

  httpd_server.on("/api/save_alarm", httpd_handler_save_alarm);
  httpd_server.on("/api/delete_alarm", httpd_handler_delete_alarm);
  httpd_server.on("/api/alarm_list", httpd_handler_alarm_list);
  httpd_server.on("/api/alarm_info", httpd_handler_alarm_info);
  httpd_server.on("/api/toggle_enable", httpd_handler_toggle_enable);


  httpd_server.on("/api/add_audio", HTTP_POST, respondOK_for_add_audio, httpd_handler_add_audio);
  httpd_server.on("/api/add_audio_path", respondOK_for_add_audio);
  httpd_server.on("/api/delete_audio", httpd_handler_remove_audio);
  httpd_server.on("/api/audio_list", httpd_handler_audio_list);

  // httpd_server.on("/upload.html",                    // 如果客户端通过upload页面
  //                   HTTP_POST,                         // 向服务器发送文件(请求方法POST)
  //                   respondOK,                         // 则回复状态码 200 给客户端
  //                   handleFileUpload);                 // 并且运行处理文件上传函数

  httpd_server.onNotFound(httpd_handler_other);

  httpd_server.enableCrossOrigin(true);
  httpd_server.enableCORS(true);

  httpd_server.begin();  // 启动网站服务

  Serial_Loginfo("HTTP server started");
}

void respondOK_for_add_audio(){
 String title = httpd_server.arg("title");
  String path = httpd_server.arg("path");

  Serial.println("File title: " + title); // 通过串口监视器输出上传文件的名称
  Serial.println("File path: " + path); // 通过串口监视器输出上传文件的名称

  if(path.length() > 0){

    fsUploadFilePath = path;
    write_bell_audio_to_memory(title, fsUploadFilePath);
    write_bell_audio_to_spiffs();

    httpd_server.send(
      200,
      "text/html;charset=utf-8",
      "OK"
    );

  }
}

void httpd_set_header_cros() {
  //允许访问所有域
  // httpd_server.sendHeader("Access-Control-Allow-Origin", "*");
  // httpd_server.sendHeader("Access-Control-Allow-Credentials", "true");
  // httpd_server.sendHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
  // httpd_server.sendHeader("Access-Control-Allow-Headers", "*");
}


// 初始化 wifi 连接
void setup_wifi_connect() {
  Serial_Loginfo("Init Wi-Fi 连接");


  if (wifi_ssid.length() > 0) {
    //连接WIFI
    WiFi.begin(wifi_ssid.c_str(), wifi_pwd.c_str());
  }
}



bool GetLocalTime(struct tm *info, uint32_t ms) {
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
void setup_net_time() {
  // Serial_Loginfo("Init 网络时间");

  // while(WiFi.status() != WL_CONNECTED){
  //   delay(1000);
  // }

  // 放置到 wifi 连接以后
  // timeClient.begin();
  // timeClient.setTimeOffset(28800); //+1区，偏移3600，+8区，偏移3600*8
}

// 初始化 校对时间
void setup_proof_time() {
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
void start_proof_time() {

  Serial_Loginfo("Init 时间校对");

  proof_time_inited = false;
  proof_time_running = true;

  stepper->rotateAsync(STEPS_PER_ROTATION * 24);
}

// 自动校时结束
void prood_time_after() {
  proof_time_inited = true;
  proof_time_running = false;

  int hours = get_hours_from_current_timestamp();
  int minutes = get_minutes_from_current_timestamp();
  int seconds = get_seconds_from_current_timestamp();

  Serial_Loginfo("时间校对完毕");
  Serial_Loginfo("当前时间: " + String(hours) + ":" + String(minutes) + ":" + String(seconds));
}


// 固件升级
String firmware_upgrade(String binfile) {
  WiFiClient client;
  t_httpUpdate_return ret = httpUpdate.update(client, binfile);
  // Or:
  //t_httpUpdate_return ret = httpUpdate.update(client, "server", 80, "file.bin");

  String r = "";
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      r = "HTTP_UPDATE_FAILD Error (" + String(httpUpdate.getLastError()) + "): " + httpUpdate.getLastErrorString().c_str();
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

void setup_chime() {
  Serial_Loginfo("Init 报时");

  int tmp_chime_list_len = 12;

  // 每天 早上8-晚上8点 整点报时
  int st = 8;
  for (int i = 0; i < tmp_chime_list_len; i++) {
    chime_list[i][0] = st + i;
    chime_list[i][1] = 0;
  }

  chime_list_len = tmp_chime_list_len;
}

void chime_time_check_handler() {

  int hours = get_hours_from_current_timestamp();
  int minutes = get_minutes_from_current_timestamp();
  int seconds = get_seconds_from_current_timestamp();

  // Serial_Loginfo("报时检查 " + String(hours) + ":" + String(minutes));

  for (int i = 0; i < chime_list_len; i++) {
    int chime_hour = chime_list[i][0];
    int chime_minute = chime_list[i][1];
    if (chime_hour == hours && chime_minute == minutes && seconds == 0) {
      Serial_Loginfo("报时检查(触发) " + String(hours) + ":" + String(minutes));
      play_chime(hours, minutes, clock_is_12_hour);
      return;
    }
  }
}

void load_bells_item_from_spiffs(const char *title) {

  String bells_item_file_path = bells_prefix + "/" + title;

  if (SPIFFS.exists(bells_item_file_path)) {

    File bells_item_file = SPIFFS.open(bells_item_file_path, "r");

    String cont = bells_item_file.readString();

    int bells_info_num = 0;

    int last_idx = 0;
    int str_idx = 0;

    String bells_title = String(title);
    String bells_cron;
    String bells_audio;
    int bell_report;
    bool bell_once_finished;
    bool bells_enable;


    while ((str_idx = cont.indexOf("\n", last_idx)) > -1) {
      String cont_item = cont.substring(last_idx, str_idx);

      if (bells_info_num == 0) {
        // bells_title = String(cont_item);
      }
      if (bells_info_num == 1) {
        bells_cron = String(cont_item);
      }
      if (bells_info_num == 2) {
        bells_audio = String(cont_item);
      }
      if (bells_info_num == 3) {
        bell_report = cont_item.toInt();
      }
      if (bells_info_num == 4) {
        bell_once_finished = cont_item.toInt();
      }
      if (bells_info_num == 5) {
        bells_enable = cont_item == "1" ? 1 : 0;
      }

      last_idx = str_idx + 1;
      bells_info_num++;
    }

    String bells_enable_str = cont.substring(last_idx, cont.length());
    bells_enable = bells_enable_str == "1" ? 1 : 0;

    if (bells_title != NULL && bells_cron != NULL) {

      BellItem *bellItem = new BellItem(bells_title.c_str(), bells_cron.c_str(), bells_enable);
      bellItem->set_audio(bells_audio.c_str());
      bellItem->set_report(bell_report);
      bellItem->set_once_finished(bell_once_finished);

      bellManager->addBell(bellItem, false);

      Serial_Loginfo(String("load_bells_item_from_spiffs add bell: ") + bells_title);
    }
  }else{
    Serial_Loginfo(String("load_bells_item_from_spiffs is not exists ") + title);
  }
}

void load_bells_config_from_spiffs() {

  if (SPIFFS.exists(bells_list_info)) {

    File bells_list_info_file = SPIFFS.open(bells_list_info, "r");

    String cont = bells_list_info_file.readString();

    Serial_Loginfo("load_bells_config_from_spiffs list.txt content:");
    Serial_Loginfo(cont.c_str());

    int bells_titles_num = 0;

    int last_idx = 0;
    int str_idx = 0;

    if(cont.indexOf("\n", last_idx) == -1){
      if(cont.length() > 0){
        load_bells_item_from_spiffs(cont.c_str());
      }
    }else{
      while ((str_idx = cont.indexOf("\n", last_idx)) > -1) {
        String title_item = cont.substring(last_idx, str_idx);

        Serial_Loginfo(String("title_item") + title_item);

        load_bells_item_from_spiffs(title_item.c_str());

        last_idx = str_idx + 1;
        bells_titles_num++;
      }

      Serial_Loginfo(String("last_idx ") + last_idx);
      String title_item = cont.substring(last_idx, cont.length());
      Serial_Loginfo(String("title_item ") + title_item);
      load_bells_item_from_spiffs(title_item.c_str());

    }
    bells_list_info_file.close();
  }else{
    Serial_Loginfo("load_bells_config_from_spiffs list.txt not exists");
  }
}

void birthday_handler() {

  int month = get_month_from_current_timestamp();
  int month_day = get_month_day_from_current_timestamp();
  int hour = get_hours_from_current_timestamp();
  int minute = get_minutes_from_current_timestamp();
  int seconds = get_seconds_from_current_timestamp();

  if(birthday == -1){ 
    return;
  }

  int birthday_month = get_month_from_timestamp((time_t *)&birthday);
  int birthday_month_day = get_month_day_from_timestamp((time_t *)&birthday);
  int birthday_hour = get_hour_from_timestamp((time_t *)&birthday);
  int birthday_minute = get_minute_from_timestamp((time_t *)&birthday);
  int birthday_seconds = get_second_from_timestamp((time_t *)&birthday);

  if (month == birthday_month
      && month_day == birthday_month_day
      && (hour == 8 || hour == 20)
      && minute == 1
      && seconds == 0) {
    start_audio("http://mithub.oss-cn-beijing.aliyuncs.com/data/hollow_clock/rings/bells/happy_birthday.mp3");
  }
}

void setup_bells() {
  Serial_Loginfo("Init 闹钟");

  bellManager = new BellManager();

  Serial_Loginfo("Load bells config from spiffs");
  // 从 spiffs 中 获取 bell 配置信息
  load_bells_config_from_spiffs();

  // 从 spiffs 中 获取 bell audio 配置信息
  load_audio_config_to_memory();
}



// 初始化固件升级
void setup_firmware_upgrade() {

  // ESP8266 配置 废弃


  // The line below is optional. It can be used to blink the LED on the board during flashing
  // The LED will be on during download of one buffer of data from the network. The LED will
  // be off during writing that buffer to flash
  // On a good connection the LED should flash regularly. On a bad connection the LED will be
  // on much longer than it will be off. Other pins than LED_BUILTIN may be used. The second
  // value is used to put the LED on. If the LED is on with HIGH, that value should be passed
  // httpUpdate.setLedPin(LED_BUILTIN, LOW);

  // Add optional callback notifiers
  // httpUpdate.onStart(firmware_upgrade_started);
  // httpUpdate.onEnd(firmware_upgrade_finished);
  // httpUpdate.onProgress(firmware_upgrade_progress);
  // httpUpdate.onError(firmware_upgrade_error);
}


float get_battery_voltage(){

   int analogValue = analogRead (BAT_ADC_PIN);

  // float input_voltage = (analogValue * 5.0) / 1024.0; 

  float input_voltage = (analogValue / 4095) * BAT_VOLTAGE_MAX; 

  return input_voltage;

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

  // 初始化spiffs
  setup_spiffs();

  // 初始化spiffs
  setup_spiffs_config();

  // 初始化 音频 输出
  setup_audio();

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

  // 初始化 铃声
  setup_chime();

  // 初始化 铃声
  setup_bells();

  Serial_Loginfo("setup finished.");
}

// 主循环
void loop() {

  // unsigned long crt_millis = millis();
  // unsigned long prv_millis = current_millis;
  // save_run_millis(crt_millis);


  // 如果没有报时 请求在执行  音频 请求
  if (!chime_handler()) {
    // 处理 音频 请求
    audio_handler();
  }

  // 处理 获取时间 请求
  get_network_time_handler();

  // 处理 dns 请求
  dns_server.processNextRequest();

  // 处理 httpd 请求
  httpd_server.handleClient();


  // Serial_Loginfo("Running Time: " + String(current_millis));
}


