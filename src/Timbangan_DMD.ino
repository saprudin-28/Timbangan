
#include <DMD.h> // dot matrik
#include "SystemFont5x7.h"
#include "Arial_Black_16.h"
#include "Arial14.h"
#include <HX711_ADC.h>
#include <SPI.h> //SPI.h must be included as DMD is written by SPI (the IDE complains otherwise)
#include <TimerOne.h>
#include <Wire.h>
#include <JC_Button.h>
#include "RTClib.h"

#define TIMER_STATE_ADDR 0
#define TIMER_MIN_ADDR 1
#define TIMER_HOUR_ADDR 2
#define BRIGHTNESS_ADDR 4

RTC_DS1307 rtc;

// Fire up the DMD library as dmd
#define OE 9
#define DISPLAYS_ACROSS 3
#define DISPLAYS_DOWN 1
DMD dmd(DISPLAYS_ACROSS, DISPLAYS_DOWN);

const int HX711_dout = 2; // mcu > HX711 dout pin
const int HX711_sck = 3;  // mcu > HX711 sck pin

HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int BUZZER = 4;
const int UP_BTN = A0;
const int DOWN_BTN = A2;
const int SEL_BTN = A1;

Button up_btn(UP_BTN);
Button sel_btn(SEL_BTN);
Button down_btn(DOWN_BTN);

const int btn_delay = 35; // tunda waktu nada / buzzer
boolean btn_press = false;
boolean curr_sel_btn;
boolean last_sel_btn;
unsigned long press_time_btn;
int long_press_time = 1500;
int btn = 0;
bool sel_state;

int brightness = 0; // this default brightnes value

void ScanDMD()
{
  dmd.scanDisplayBySPI();
  Timer1.setPwmDuty(OE, brightness);
}

unsigned long t_btn = 0;
unsigned long last_t_btn = 0;
unsigned long t_data = 0; // waktu untuk membaca data
unsigned long last_t_data = 0;
unsigned long t_lock = 0;
unsigned long interval_data = 500; // tunda waktu untuk membaca data
const int interval_lock = 3;       // tunda waktu saat akan di kunci
unsigned long interval_btn = 200;  // waktu untuk long press button
boolean newDataReady = false;

enum states_t
{
  STANDBY_DISP,
  SETTINGS,
  SETT_TIMER,
  SETT_BRIGHTNESS
};
enum settings
{
  TIMER_SETT,
  BRIGHTNESS_SETT
};

static states_t STATE;

bool timer_state = false; // status timer
bool lock_state = false;  // status kunci kiloan
bool start_state = false;
unsigned int lock_count = 0;

float loadLock = 0.00;
float loadNow = 0.00;
float loadData = 0.00;

int hour_timer; // Timer satuan jam
int min_timer; // Timer satuan menit
int sec_timer; // Timer Satuan detik
int hour_set; // pengaturan jam Timer
int minut_set; // pengaturan menit timer
int hour_now; // jam sekarang /via rtc
int minute_now; // menit sekarang / via rtc
int sec_now;  // detik sekarang / via rtc
int second_now; // detik sekarang / via rtc
int hour_timer_s; // jam timer (satuan) untuk di tampilkan di dmd
int hour_timer_p; // jam timer (puluhan) untuk di tampilkan di dmd
int min_timer_s; // menit timer (satuan) untuk di tampilkan di dmd
int min_timer_p; // menit timer (puluhan) untuk di tampilkan di dmd
int sisa_hour; // sisa waktu (jam) 
int sisa_min; // sisa waktu (menit)
int sisa_sec; // sisa waktu (detik)
int step_min = 5; // step menit di pengaturan timer
boolean sec_state = false; // status detik

unsigned long f_time;
unsigned long last_f_time;
unsigned long last_f;
unsigned long f;
boolean c;

void setup()
{
  Serial.begin(115200);
  pinMode(BUZZER, OUTPUT);
  pinMode(UP_BTN, INPUT_PULLUP);
  pinMode(DOWN_BTN, INPUT_PULLUP);
  pinMode(SEL_BTN, INPUT_PULLUP);
  //pinMode(OE, OUTPUT);
  //digitalWrite(OE, HIGH);
  start_state = true; // status startup ketika timer tidak aktip
  lock_state = false; // status kunci timbangan
  up_btn.begin();
  sel_btn.begin();
  down_btn.begin();

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }

  if (!rtc.isrunning())
  {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  float calibrationValue;
  calibrationValue = 56.75; // nilai kalibrasi

  LoadCell.begin();
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  }
  else
  {
    LoadCell.setCalFactor(calibrationValue); // set calibration factor (float)
    Serial.println("Startup is complete");
  }

  /*
    while (!LoadCell.update());
    Serial.print("Calibration value: ");
    Serial.println(LoadCell.getCalFactor());
    Serial.print("HX711 measured conversion time ms: ");
    Serial.println(LoadCell.getConversionTime());
    Serial.print("HX711 measured sampling rate HZ: ");
    Serial.println(LoadCell.getSPS());
    Serial.print("HX711 measured settlingtime ms: ");
    Serial.println(LoadCell.getSettlingTime());
  */

  Timer1.initialize(5000); // period in microseconds to call ScanDMD. Anything
  // longer than 5000 (5ms) and you can see flicker.
  Timer1.attachInterrupt(ScanDMD); // attach the Timer1 interrupt to ScanDMD
  // which goes to dmd.scanDisplayBySPI()
  Timer1.pwm(OE, 50);

  updateRam();
  updateTimer();

  // clear/init the DMD pixels held in RAM
  dmd.clearScreen(true); // true is normal (all pixels off), false is negative
  // (all pixels on)
  if (!timer_state)
    startUp(); // startup aktip ketika timer tidak aktip
}

void loop()
{
  sel_btn.read();
  up_btn.read();
  down_btn.read();

  if (!sel_btn.isPressed())
    sel_state = false;

  switch (STATE)
  {
  case STANDBY_DISP:
    standby_display();
    if (sel_btn.wasPressed()) // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> jika Tombol Di tekan akan di nol kan
    {
      LoadCell.tareNoDelay();
      pressBuzz();
    }
    if (sel_btn.pressedFor(long_press_time) && !sel_state) // >>>>>>>>>>>>>>>>>>>>>>>>> jika tombol di tekan Lama masuk ke setting
    {
      longPressBuzz();
      STATE = SETTINGS;
      dmd.clearScreen(true);
      sel_state = true;
    }
    break;

  case SETTINGS:
    static settings SETT;

    switch (SETT)
    {
    case TIMER_SETT:
      dmd.selectFont(Arial_Black_16);
      dmd.drawString(22, 0, "Timer", 5, GRAPHICS_NORMAL);

      if (up_btn.wasReleased())
      {
        pressBuzz();
        SETT = BRIGHTNESS_SETT;
        dmd.clearScreen(true);
      }
      if (down_btn.wasReleased())
      {
        pressBuzz();
        SETT = BRIGHTNESS_SETT;
        dmd.clearScreen(true);
      }
      if (sel_btn.wasPressed()) //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Tombol sel ditekan
      {
        pressBuzz();
        hour_set = hour_timer;
        minut_set = min_timer;
        STATE = SETT_TIMER;
        dmd.clearScreen(true);
      }
      break;

    case BRIGHTNESS_SETT:
      dmd.selectFont(Arial_Black_16);
      dmd.drawString(3, 0, "Kecerahan", 9, GRAPHICS_NORMAL);
      if (up_btn.wasReleased())
      {
        pressBuzz();
        SETT = TIMER_SETT;
        dmd.clearScreen(true);
      }
      if (down_btn.wasReleased())
      {
        pressBuzz();
        SETT = TIMER_SETT;
        dmd.clearScreen(true);
      }
      if (sel_btn.wasPressed()) //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Tombol sel ditekan
      {
        pressBuzz();
        STATE = SETT_BRIGHTNESS;
        dmd.clearScreen(true);
      }
      break;
    }
    if (sel_btn.pressedFor(long_press_time) && !sel_state) // >>>>>>>>>>>>>>>>>>>> TOMBOL Di takan LAMA keluar dari pengatiran
    {
      longPressBuzz();
      STATE = STANDBY_DISP;
      dmd.clearScreen(true);
      sel_state = true;
    }
    break;

  case SETT_TIMER:

    if (hour_set < 10)
      hour_timer_s = hour_set;
    if (hour_set >= 10)
      hour_timer_s = int(hour_set) % 10;
    if (hour_set >= 10 && hour_set < 20)
      hour_timer_p = int(hour_set / 10);
    if (hour_set < 10)
      hour_timer_p = 0;

    if (minut_set < 10)
      min_timer_s = minut_set;
    if (minut_set >= 10)
      min_timer_s = int(minut_set) % 10;
    if (minut_set >= 10)
      min_timer_p = int(minut_set / 10);
    if (minut_set < 10)
      min_timer_p = 0;

    dmd.selectFont(Arial_Black_16);
    dmd.drawChar(22, 0, hour_timer_p + 48, GRAPHICS_NORMAL);
    dmd.drawChar(32, 0, hour_timer_s + 48, GRAPHICS_NORMAL);
    dmd.drawChar(42, 0, ':', GRAPHICS_NORMAL);
    dmd.drawChar(47, 0, min_timer_p + 48, GRAPHICS_NORMAL);
    dmd.drawChar(57, 0, min_timer_s + 48, GRAPHICS_NORMAL);

    if (sel_btn.pressedFor(long_press_time) && !sel_state) // <>>>>>>>>>>>>>>>>>>>>>>>>>>>>> simpan data
    {
      sel_state = true;
      longPressBuzz();

      rtc.writenvram(TIMER_HOUR_ADDR, hour_set);
      rtc.writenvram(TIMER_MIN_ADDR, minut_set);
      updateRam();
      startTime();
      Serial.print("Timer :");
      Serial.print(hour_timer);
      Serial.print(" :");
      Serial.println(min_timer);

      dmd.clearScreen(true);
      STATE = STANDBY_DISP;
    }
    if (up_btn.wasReleased())
    {
      pressBuzz();
      dmd.clearScreen(true);
      Serial.print("menit ");
      Serial.println(minut_set);
      Serial.print("jam ");
      Serial.println(hour_set);
      Serial.print("satuan menit ");
      Serial.println(min_timer_s);
      minut_set += step_min;
      if (minut_set >= 60)
      {
        minut_set = 0;
        hour_set = hour_set + 1;
      }
      rtc.writenvram(TIMER_HOUR_ADDR, hour_set);
      rtc.writenvram(TIMER_MIN_ADDR, minut_set);
    }
    if (down_btn.wasReleased())
    {
      pressBuzz();
      dmd.clearScreen(true);
      Serial.print("menit ");
      Serial.println(minut_set);
      Serial.print("jam ");
      Serial.println(hour_set);
      Serial.print("satuan menit ");
      Serial.println(min_timer_s);
      minut_set -= step_min;
      if (hour_set > 0)
      {
        if (minut_set < 0)
        {
          minut_set = 60 - step_min;
          hour_set--;
        }
      }
      if (hour_set == 0)
      {
        if (minut_set <= 0)
        {
          minut_set = step_min;
          longPressBuzz();
        }
      }

      rtc.writenvram(TIMER_HOUR_ADDR, hour_set);
      rtc.writenvram(TIMER_MIN_ADDR, minut_set);
    }

    break;

  case SETT_BRIGHTNESS:

    int bright_conv = map(brightness, 0, 1023, 0, 96);
    if (up_btn.wasPressed())
    {
      brightness += 10;
      if (brightness > 1023)
        brightness = 1023;
      pressBuzz();
    }
    if (down_btn.wasPressed())
    {
      brightness -= 10;
      if (brightness <= 0)
        brightness = 10;
      pressBuzz();
    }
    if (sel_btn.pressedFor(long_press_time) && !sel_state)
    {
      dmd.clearScreen(true);
      rtc.writenvram(BRIGHTNESS_ADDR, map(brightness, 0, 1023, 0, 254));
      updateTimer();
      updateRam();
      STATE = STANDBY_DISP;
      longPressBuzz();
      sel_state = true;
    }
    if (!sel_state)
    {
      for (int t = 1; t < 6; t++)
      {
        for (int l = 1; l < bright_conv; l++)
        {
          dmd.writePixel(l, t + 4, GRAPHICS_NORMAL, 1);
          dmd.writePixel(l + 1, t + 4, GRAPHICS_NORMAL, 0);
          dmd.writePixel(l + 2, t + 4, GRAPHICS_NORMAL, 0);
        }
      }
    }
    if (sel_state)
    {
      dmd.clearScreen(true);
    }
    break;
  }
}

void standby_display()
{

  last_t_data = millis();
  last_f = millis();
  last_f_time = millis();

  tareCmd(); // data di nolkan

  // dmd.drawString(0, 0, "FC", 2 , GRAPHICS_NORMAL);
  // dmd.drawString(2, 0, "Cipetir", 7, GRAPHICS_NORMAL);

  DateTime now = rtc.now();
  if (timer_state)
  {
    // Hitung Sisa detik
    sisa_sec = 59 - now.second();

    // Hitung Sisa Menit
    if (sisa_sec == 59 && !sec_state)
    {

      sisa_min = sisa_min - 1;
      if (sisa_min < 0)
      {
        sisa_hour -= 1;
        sisa_min = 59;
      }
      sec_state = true;
    }
    if (sisa_sec == 0)
      sec_state = false;

    if (sisa_hour == 0 && sisa_min == 0 && sisa_sec == 0) // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Timer Habis
    {
      digitalWrite(BUZZER, HIGH);
      timer_state = false;
      rtc.writenvram(TIMER_STATE_ADDR, false);
      delay(2000);
      digitalWrite(BUZZER, LOW);

      dmd.clearScreen(true);
      dmd.selectFont(Arial_Black_16);
      dmd.drawMarquee("Waktu Habis Silahkan Angkat Pancingan", 37, (32 * DISPLAYS_ACROSS) - 1, 0);
      long start = millis();
      long timer = start;
      boolean ret = false;
      while (!ret)
      {
        if ((timer + 30) < millis())
        {
          ret = dmd.stepMarquee(-1, 0);
          timer = millis();
        }
      }
      while (!timer_state)
      {

        dmd.drawString(4, 1, "FC CIPETIR", 10, GRAPHICS_NORMAL);

        sel_btn.read();
        if (sel_btn.wasReleased()) // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Tombol tengah Di tekan
        {
          LoadCell.tareNoDelay();
          digitalWrite(BUZZER, HIGH);
          delay(btn_delay);
          digitalWrite(BUZZER, LOW);
          if (!timer_state)
          {
            startTime();
            digitalWrite(BUZZER, HIGH);
            delay(1000);
            digitalWrite(BUZZER, LOW);
            start_state = false;
            dmd.clearScreen(true);
          }
        }
        if (sel_btn.pressedFor(long_press_time) && !sel_state)
        {
          dmd.clearScreen(true);
          longPressBuzz();
          STATE = SETTINGS;
          sel_state = true;
        }
      }
    }
    int s_hour;
    int p_hour;
    if (sisa_hour < 10)
    {
      s_hour = sisa_hour;
      p_hour = 0;
    }
    else if (sisa_hour >= 10)
    {
      s_hour = sisa_hour % 10;
      p_hour = sisa_hour / 10;
    }
    int s_min;
    int p_min;
    if (sisa_min < 10)
    {
      s_min = sisa_min;
      p_min = 0;
    }
    else if (sisa_min >= 10)
    {
      s_min = sisa_min % 10;
      p_min = sisa_min / 10;
    }
    int p_sec;
    int s_sec;
    if (sisa_sec < 10)
    {
      s_sec = sisa_sec;
      p_sec = 0;
    }
    else if (sisa_sec >= 10)
    {
      s_sec = sisa_sec % 10;
      p_sec = sisa_sec / 10;
    }

    dmd.drawLine(54, 0, 54, 15, GRAPHICS_NORMAL);
    dmd.drawLine(55, 0, 55, 15, GRAPHICS_NORMAL);
    dmd.selectFont(Arial_14);
    dmd.drawChar(2, 2, p_hour + 48, GRAPHICS_NORMAL);
    dmd.drawChar(10, 2, s_hour + 48, GRAPHICS_NORMAL);
    dmd.drawChar(20, 2, p_min + 48, GRAPHICS_NORMAL);
    dmd.drawChar(28, 2, s_min + 48, GRAPHICS_NORMAL);
    dmd.drawChar(38, 2, p_sec + 48, GRAPHICS_NORMAL);
    dmd.drawChar(46, 2, s_sec + 48, GRAPHICS_NORMAL);

    if (sec_now != sisa_sec)
    {
      if (c)
      {
        //  dmd.clearScreen(true);
        if (s_sec == 1)
        {
          // dmd.clearScreen(true);
        }
        c = false;

        Serial.print("jam : ");
        Serial.print(now.hour());
        Serial.print(" : ");
        Serial.print(now.minute());
        Serial.print(" : ");
        Serial.println(now.second());

        Serial.print("Timer : ");
        Serial.print(sisa_hour);
        Serial.print(" : ");
        Serial.print(sisa_min);
        Serial.print(" : ");
        Serial.println(sisa_sec);
      }

      if (last_f - f >= 500)
      {
        for (int t = 0; t < 2; t++)
        {
          for (int l = 0; l < 2; l++)
          {
            dmd.writePixel(17 + l, 4 + t, GRAPHICS_NORMAL, 1);
            dmd.writePixel(17 + l, 10 + t, GRAPHICS_NORMAL, 1);
            dmd.writePixel(35 + l, 4 + t, GRAPHICS_NORMAL, 1);
            dmd.writePixel(35 + l, 10 + t, GRAPHICS_NORMAL, 1);
          }
        }
        if (sisa_sec <= 10 && sisa_sec > 0 && sisa_min == 0 && sisa_hour == 0)
          digitalWrite(BUZZER, HIGH);

        f = last_f;
        // dmd.clearScreen(true);
      }
      if (last_f_time - f_time >= 500)
      {

        c = true;
        sec_now = sisa_sec;
      }
    }
    else if (sec_now == sisa_sec)
    {

      f_time = last_f_time;
      if (sisa_sec <= 10 && sisa_sec > 0 && sisa_min == 0 && sisa_hour == 0)
      {
        digitalWrite(BUZZER, LOW);
      }
      /*
      for (int t = 0; t < 2; t++)
      {
        for (int l = 0; l < 2; l++)
        {
          dmd.writePixel(17 + l, 4 + t, GRAPHICS_NORMAL, 0);
          dmd.writePixel(17 + l, 10 + t, GRAPHICS_NORMAL, 0);
          dmd.writePixel(35 + l, 4 + t, GRAPHICS_NORMAL, 0);
          dmd.writePixel(35 + l, 10 + t, GRAPHICS_NORMAL, 0);
        }
      } */
    }

    if ((sisa_hour == 0 && sisa_min == 5 && sisa_sec == 0) || (sisa_hour == 0 && sisa_min == 0 && sisa_sec == 12))
    {
      digitalWrite(BUZZER, HIGH);
      delay(1000);
      digitalWrite(BUZZER, LOW);
    }
  }

  else if (!timer_state)
  {
    // dmd.selectFont(Arial_14);
    dmd.drawString(0, 1, "FC", 2, GRAPHICS_NORMAL);
    dmd.drawString(19, 2, "Cipetir", 7, GRAPHICS_NORMAL);
    if (up_btn.isPressed())
    {
      if (down_btn.wasPressed())
      {
        dmd.clearScreen(true);
        updateRam();
        if (hour_timer > 0 && min_timer > 0)
          startTime();
      }
    }
  }

  // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>tampilan kilogram
  dmd.selectFont(Arial_Black_16);
  // baca data dari hx711

  newDataReady = false;
  if (LoadCell.update())
    newDataReady = true;
  if (last_t_data - t_data >= interval_data)
  {

    loadData = LoadCell.getData() / 1000;
    int s = (int)loadData;
    int pk = int(float(loadData * 10)) % 10;
    int sk = int(float(loadData * 100)) % 10;
    if (lock_count >= interval_lock)
      lock_state = true;
    if (!lock_state && loadData < 10)
    {
      loadNow = s + (float(pk) / 10) + (float(sk) / 100);
    }
    if (lock_state && lock_count == interval_lock)
    {
      for (byte i = 0; i < 5; i++)
      {
        digitalWrite(BUZZER, HIGH);
        delay(100);
        digitalWrite(BUZZER, LOW);
        delay(100);
      }
      // dmd.clearScreen(true);
    }

    if (loadNow > 0.02)
    {
      if (loadNow != loadLock)
      {
        lock_count = 0;
        loadLock = loadNow;
      }
      if (loadNow == loadLock)
      {
        lock_count++;
      }
    }
    newDataReady = false;
    t_data = last_t_data;
  }

  loadNow = abs(loadNow); // ubah nilai menjadi absolut
  int s1 = (int)loadNow;
  int pk1 = int(float(loadNow * 10)) % 10;
  int sk1 = int(float(loadNow * 100)) % 10;
  dmd.drawChar(58, 1, (s1 + 48), GRAPHICS_NORMAL);
  dmd.drawChar(67, 1, 46, GRAPHICS_NORMAL);
  dmd.drawChar(71, 1, (pk1 + 48), GRAPHICS_NORMAL);
  dmd.drawChar(81, 1, (sk1 + 48), GRAPHICS_NORMAL);
  dmd.selectFont(SystemFont5x7);
  dmd.drawChar(90, 0, 'K', GRAPHICS_NORMAL);
  dmd.drawChar(90, 8, 'G', GRAPHICS_NORMAL);
}

void startUp()
{
  dmd.clearScreen(true);
  dmd.selectFont(Arial_Black_16);
  dmd.drawMarquee("Selamat datang di Pemancingan FC Cipetir", 40, (32 * DISPLAYS_ACROSS) - 1, 0);
  long start = millis();
  long timer = start;
  boolean ret = false;
  while (!ret)
  {
    if ((timer + 30) < millis())
    {
      ret = dmd.stepMarquee(-1, 0);
      timer = millis();
    }
  }
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);
  delay(100);
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);
  delay(100);

  while (start_state)
  {
    dmd.drawString(4, 1, "FC CIPETIR", 10, GRAPHICS_NORMAL);

    sel_btn.read();
    if (sel_btn.wasReleased()) // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Tombol tengah Di tekan
    {
      LoadCell.tareNoDelay();
      digitalWrite(BUZZER, HIGH);
      delay(btn_delay);
      digitalWrite(BUZZER, LOW);
      if (!timer_state)
      {
        startTime();
        digitalWrite(BUZZER, HIGH);
        delay(1000);
        digitalWrite(BUZZER, LOW);
        start_state = false;
        dmd.clearScreen(true);
      }
    }
    if (sel_btn.pressedFor(long_press_time))
    {
      longPressBuzz();
      STATE = SETTINGS;
      sel_state = true;
      start_state = false;
      dmd.clearScreen(true);
    }
  }
}

void tareCmd()
{
  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0)
  {
    char inByte = Serial.read();
    if (inByte == 't')
    {
      LoadCell.tareNoDelay();
      lock_state = false;
    }
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true)
  {
    Serial.println("Tare complete");

    lock_state = false;
    lock_count = 0;
    for (byte i = 0; i < 2; i++)
    {
      digitalWrite(BUZZER, HIGH);
      delay(50);
      digitalWrite(BUZZER, LOW);
      delay(50);
    }
    dmd.clearScreen(true);
  }
}

void updateRam()
{
  brightness = map(rtc.readnvram(BRIGHTNESS_ADDR), 0, 225, 0, 1023);
  hour_timer = rtc.readnvram(TIMER_HOUR_ADDR);
  min_timer = rtc.readnvram(TIMER_MIN_ADDR);
  timer_state = rtc.readnvram(TIMER_STATE_ADDR);
  if (rtc.readnvram(TIMER_STATE_ADDR) > 1)
    rtc.writenvram(TIMER_STATE_ADDR, 0);
  Serial.print("Kecerahan = ");
  Serial.println(brightness);
  Serial.print("Jam Teimr  = ");
  Serial.println(hour_timer);
  Serial.print("menit Timer = ");
  Serial.println(min_timer);
  Serial.print("status Timer = ");
  Serial.println(timer_state);
}
void pressBuzz()
{
  digitalWrite(BUZZER, HIGH);
  delay(btn_delay);
  digitalWrite(BUZZER, LOW);
}
void longPressBuzz()
{
  digitalWrite(BUZZER, HIGH);
  delay(btn_delay * 5);
  digitalWrite(BUZZER, LOW);
}

void startTime()
{
  rtc.adjust(DateTime(2000, 1, 1, 0, 0, 0)); // atur jam jadi jam : 00, menit : 00
  timer_state = true;
  rtc.writenvram(TIMER_STATE_ADDR, true);
  updateRam();
  sisa_hour = hour_timer;
  sisa_min = min_timer - 1;
}

float loadCell()
{
  float data;
  newDataReady = false;
  if (LoadCell.update())
    newDataReady = true;
  if (last_t_data - t_data >= interval_data)
  {

    data = LoadCell.getData() / 1000;
    Serial.println(data);
    newDataReady = false;
    t_data = last_t_data;

    return data;
  }
}

void updateTimer()
{
  DateTime sekarang = rtc.now();
  /* if (((hour_timer - now.hour()) < 0) && (min_timer - now.minute()) < 0  )
    {
     timer_state = false;
     rtc.writenvram(TIMER_STATE_ADDR, false);
    } */
  // baca kondisi timer, jika tidak aktip langsung di teruskan ke startup

  sisa_hour = hour_timer - sekarang.hour();
  sisa_min = min_timer - sekarang.minute() - 1;
  if (sisa_min < 0)
    sisa_hour -= 1;

  if (sisa_hour < 0)
  {
    sisa_hour = 0;
    if (sisa_min < 0)
      sisa_min = 0;
  }
  if (sisa_hour >= 0)
  {
    if (sisa_min < 0)
      sisa_min = 59 - sekarang.minute();
  }

  if (hour_timer == sekarang.hour())
  {
    if (min_timer < sekarang.minute())
    {
      timer_state = false;
      rtc.writenvram(TIMER_STATE_ADDR, false);
    }
  }
  if (hour_timer < sekarang.hour())
  {
    timer_state = false;
    rtc.writenvram(TIMER_STATE_ADDR, false);
  }
}