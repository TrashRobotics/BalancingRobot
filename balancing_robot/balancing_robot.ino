/*
Скетч к проекту балансирующего робота
Github: https://github.com/TrashRobotics/BalancingRobot
Страница канала на YouTube: https://www.youtube.com/channel/UCHRTaqr8KSCfyo2_CLH-yfg
Автор: TrashRobotics
*/
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "ArduinoJson.h"
#include <EEPROM.h>
#include "SSD1306Wire.h" 
#include "MPU6050.h"
#include "Wire.h"
#include "String.h" 
#include <L298NX2.h>
#include <Ultrasonic.h>
#include "sources.h"    // изображения, web-страницы, css и т.д.


#define MPU_ADRESS 0x68     // адреса I2C устройств 
#define SCREEN_ADDRESS 0x3C // 

#define EEPROM_RW_ADDR  0x00 // адресс чтения-записи в eeprom
#define EEPROM_INIT_KEY 0x04  // ключ проверки - проводилась ли первоначальная инициализация eeprom

#define VOLTAGE_REF 3.3f    // питающее esp напряжение 
#define VOLTAGE_GAIN 3.94f  // коэффициент передачи делителя напряжения 13V <-> 3.3V
#define VOLTAGE_OFFSET 60   // 60  // ошибка измерения аналогового выхода 
#define MIN_SUPPLY_VOLTAGE 6.5f // минимальное напряжение АКБ
#define LOW_VOLTAGE_WARNING_MSG_PERIOD 15000 // в миллисекундах

#define SDA_PIN D1  // пины шины I2C
#define SCL_PIN D2  //
#define VOLTAGE_PIN A0  // пин для считывания напряжения с li-ion 
#define US_TRIG_PIN D3  // общий пин для входа trig c ультразвуковых датчиков 
#define US_ECHO1_PIN D5 // пины для выходов echo с ультразвуковых датчиков   
#define US_ECHO2_PIN D6 //
#define ENA_PIN D7  // пины драйвера моторов l298n 
#define ENB_PIN D8  //
#define IN1_PIN D0 //
#define IN2_PIN D4  //
#define IN3_PIN D9  //
#define IN4_PIN D10  //

#define MAX_P_VALUE 30  // максимальное значение пропорционального коэффициента (в основном нужно для слайдера в web странице)
#define MAX_I_VALUE 300  // максимальное значение интегрального коэффициента (в основном нужно для слайдера в web странице)
#define MAX_D_VALUE 5  // максимальное значение дифференциального коэффициента (в основном нужно для слайдера в web странице)

// экземпляры используемых классов
ESP8266WebServer webServer(80);
IPAddress ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
StaticJsonDocument<400> jsondoc;

MPU6050 mpu(MPU_ADRESS);
SSD1306Wire display(SCREEN_ADDRESS, SDA_PIN, SCL_PIN);
L298NX2 motors(ENA_PIN, IN1_PIN, IN2_PIN, ENB_PIN, IN3_PIN, IN4_PIN);
Ultrasonic ultrasonicFront(US_TRIG_PIN, US_ECHO1_PIN);
Ultrasonic ultrasonicBack(US_TRIG_PIN, US_ECHO2_PIN);

// данные точки доступа
const char *ssid = "balancingbot";
const char *password = "123456789";

// настраиваемые коэффициенты (можно впоследствие вынести в web страницу)
float alfaAg = 0.02; // коэффициент ФНЧ угла отклонения  
float kP = 12.0;  // пропорциональный коэффициент PID-регулятора
float kI = 0.0;   // интегральный коэффициент PID-регулятора
float kD = 0.01; // дифференциальный коэффициент PID-регулятора
float target = -2.0; // базовый угол удержания платформы в градусах
float targetDeadZone = 1.0; // мертвая зона около угла удержания
float moveAdditionalAngle = 3.0;  // добавочный угол, за счет которого производится передвижение

bool isAfraid = true;  // флаг, используются ли датчики расстояния, для определения препятствий 
uint16_t minAfraidDistance = 10;  // минимальное расстояние, после которого платформа отъезжает от препятствия (в сантиметрах)

int16_t accXoffset = -80;   // компенсация смещений IMU датчика
int16_t accYoffset = -80;   //
int16_t accZoffset = -620;   //

int16_t gyroXoffset = 15;  //
int16_t gyroYoffset = -350;  //
int16_t gyroZoffset = 40;  //

uint8_t minAbsSpeed = 100;  // минимальая скорость, подаваемая на моторы: вместо [0-255] <-> [minAbsSpeed-255]
float stopAngle = 30;   // наибольший угол отклонения в обе стороны, после которого робот останавливается
uint8_t movePeriod = 10; // период обовления управляющих сигналов в миллисекундах // если делать меньше точка доступа глохнет

float speedFactorA = 0.91;  // дополнительные параметры компенсирующие разницу характеристик моторов
float speedFactorB = 0.99;  //
float rotateAdditionalSpeed = 50; // добавочная скорость, суммирующаяся с основной, за счет которой производится поворот на месте 

// сохраняемые в eeprom данные
struct StoredData{  
  uint8_t key;  // проверка на первую инициализацию: если key != EEPROM_INIT_KEY, то данные еще не были записаны
  float kP;
  float kI;
  float kD;
};

// состояния конечного автомата
enum FsmStates
{
  INITIALIZATION, // инициализация системы
  LOW_VOLTAGE,  // прерывает инициализацию и работу, уходит в полуспящий режим
  BALANCING, // обычный режим работы
  ERROR // если появилась какая-то ошибка
};
 
// глобальные переменные
FsmStates state = FsmStates::INITIALIZATION;
uint32_t lowVoltageTimer = 0; // таймер показа сообщения о низком заряде
uint32_t pidTimer = 0;  // таймер обновления управляющих сигналов
bool afraidFlag = false;  // флаг - было ли замечено препятствие
int16_t ax, ay, az;   // вынесены как глобальные, чтоб можно было позже сделать калибровку через web страницу
int16_t wx, wy, wz;   // 
float additionalAngle = 0.f; // действующая добавка к углу удержания
float additionalSpeed = 0.f;  // действующая добавка к скорости
float angle = 0.f;  // текущий угол наклона, снятый с MPU


// предъобявление некоторых функций с параметрами по умолчанию из-за костылей arduino IDE
void smoothProgressBar(uint8_t progressStart, uint8_t progressEnd, String text, uint8_t dt=30);
float pid(float input, float trgt, float kp, float ki, float kd, float dt, bool reset=false);


void setup() {
  // настройка переферии
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  
  Wire.begin(SDA_PIN, SCL_PIN);
}


void loop() {
  switch(state){
      case FsmStates::INITIALIZATION:{
        // инициализация дисплея
        display.init();
        display.flipScreenVertically();
        display.setFont(ArialMT_Plain_10);
        display.clear();
        display.drawXbm(32, 0, LOGO_WIDTH, LOGO_HEIGHT, logoBits); // рисуем лого
        display.display();
        delay(3000);       
  
        // рисуем полосу загрузки, которые также используются как delay
        smoothProgressBar(0, 10, "Check voltage...");
        smoothProgressBar(11, 20, "Voltage: " + String(getVoltage()) + " V");
        
        if(getVoltage() < MIN_SUPPLY_VOLTAGE){
          state = FsmStates::LOW_VOLTAGE;
          progressBar(21, "Voltage is low");
          delay(500);
          break; // останавливаем инициализацию
        }
        
        smoothProgressBar(22, 30, "Voltage is acceptable");     
        smoothProgressBar(31, 40, "MPU initialize...");
  
        mpu.initialize(); // инициализация IMU
        if(!mpu.testConnection()){
          state = FsmStates::ERROR;
          progressBar(41, "MPU connection failed");
          delay(500);
          break;
        }
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4); // настройка IMU
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500); //
        
        smoothProgressBar(42, 50, "MPU initializing successful");
        smoothProgressBar(51, 60, "EEPROM initialize...");
        
        EEPROM.begin(512);
        StoredData stval; // создаем структуру, читаемую из eeprom
        EEPROM.get(EEPROM_RW_ADDR, stval);
        
        smoothProgressBar(61, 65, "EEPROM check first initialization...");
        
        if(stval.key != EEPROM_INIT_KEY){   // если запись в eeprom никогда не производилась
          stval.key = EEPROM_INIT_KEY;
          stval.kP = kP;
          stval.kI = kI;
          stval.kD = kD;
          EEPROM.put(EEPROM_RW_ADDR, stval);  // записываем значения по умолчанию
          EEPROM.commit();
          smoothProgressBar(66, 72, "EEPROM first initialize...");
        }
        else{ // если запись уже производилась, то загружаем данные из eeprom
          kP = stval.kP;
          kI = stval.kI;
          kD = stval.kD;
          smoothProgressBar(66, 72, "EEPROM reading data...");
        }
        
        smoothProgressBar(73, 80, "WIFI AP initialize...");
        //настраиваем точку доступа
        WiFi.mode(WIFI_AP);   
        WiFi.softAPConfig(ip, gateway, subnet);
        bool result = WiFi.softAP(ssid, password);
        if (!result){
          state = FsmStates::ERROR;
          progressBar(81, "WiFi softAP failed");
          delay(500);
          break;
        }

        smoothProgressBar(82, 90, "Web server initialize...");
        // подключаем обработчики GET запросов
        webServer.on("/", handleControl);           // корневая директория - страница управления
        webServer.on("/settings", handleSettings);  // страница настроек pid-регулятора
        webServer.on("/about", handleAbout);        // страница about
        webServer.on("/general.css", handleCss);    // отдельно сервится таблица стилей
        webServer.on("/getData", handleGetData);    // обработчик запроса на получение данных html страницей
        webServer.on("/setPid", handleSetPid);      // обработчик запроса на установку значения PID коэффициентов
        webServer.on("/saveData", handleSaveData);  // обработчик запроса на сохраение текущих PID коэффициентов в eeprom
        webServer.on("/move", handleMove);          // обработчик запросов передвижения платформы
        webServer.onNotFound(handleNotFound);       
        webServer.begin();

        smoothProgressBar(91, 99, "Web server initialize...");
        
        state = FsmStates::BALANCING; //  переключаемся в рабочее состояние
        progressBar(100, "Loading is complete");
        delay(500);
        break;
      }
      
      case FsmStates::LOW_VOLTAGE:{ // переодически зажигает экран на 3 секунды, говоря, что АКБ сел
        if ((millis() > lowVoltageTimer + LOW_VOLTAGE_WARNING_MSG_PERIOD) || (lowVoltageTimer == 0)){
          displayLowVoltage();
          delay(3000);
          display.clear();
          display.display();
          lowVoltageTimer = millis();
        }
        break;
      }  
      
      case FsmStates::BALANCING:{ 
        if (!checkVoltage()){   // проверяется раз в секунду, т.к., если часто дергать аналоговый выход, падает вайвай -__- 
          moveA(0);  // останавливаем моторы
          moveB(0);
          state = FsmStates::LOW_VOLTAGE;
          delay(100);
          break;
        }

        if (millis() > pidTimer + movePeriod){
          angle = getAngleY() * RAD_TO_DEG; // получаем угол по оси Y
          float dt = ((float)(millis() - pidTimer)/1000.f); 
          float speed = 0.f;
          if (abs(angle) < stopAngle){  // если робот НЕ наклонился слишком сильно
            speed = pid(angle, target + additionalAngle, kP, kI, kD, dt);  // получаем скорость с PID-регулятора
            speed = (speed/abs(speed))*(abs(speed) + minAbsSpeed);  // добавлем к ее модулю минимальное значение
            speed = -speed; // инвертируем скорость
          }
          else{
            pid(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, true);  // сбрасываем данные pid регулятора (reset=true)
            speed = 0.f;  // скорость на ноль            
          }

          if (isAfraid){
            if (checkWallOnFront() && checkWallOnBack()){   // если препятствие и спереди и сзади, то балансируем на месте
              additionalAngle = 0.f;
              additionalSpeed = 0.f;
              afraidFlag = true;
            } else if (checkWallOnFront()) {
              additionalAngle = -moveAdditionalAngle;    // двигаемся назад
              additionalSpeed = 0.f;                    // и не поворачивваем
              afraidFlag = true;
            } else if (checkWallOnBack()) {
              additionalAngle = moveAdditionalAngle;    // двигаемся вперед
              additionalSpeed = 0.f;                    // и не поворачивваем
              afraidFlag = true;
            } else if (afraidFlag) {   // если до этого уезжали от препятствия
              additionalAngle = 0.f;
              additionalSpeed = 0.f;
              afraidFlag = false;
            }
          }

          // добавление мертвой зоны
          if ((angle <= (target + targetDeadZone/2.f)) && (angle >= (target - targetDeadZone/2.f)) && (additionalAngle == 0.f)) {
            speed = 0.f;
          }
          
          moveA(constrain(speed*speedFactorA - additionalSpeed, -255.f, 255.f));  // двигаем моторы, учитывая их разницу в характеристиках +
          moveB(constrain(speed*speedFactorB + additionalSpeed, -255.f, 255.f));  // добавляем скорости на моторы для поворота, если он есть
  
          displayFace();
          //displayMpuRaw();                        // функции для дебага
          //displayAngleAndSpeed(speed, angle);     // 
          //displayDistance(ultrasonicFront.read(), ultrasonicBack.read()); //
          
          pidTimer = millis();
        }
        break;
      }  
      
      case FsmStates::ERROR:{
        break;
      }    
      
      default: {
        break;
      }
  }
  webServer.handleClient();
}


void handleControl() {
 webServer.send_P(200, "text/html", controlHtml); // отправляем веб страницу
}


void handleSettings() {
 webServer.send_P(200, "text/html", settingsHtml); // отправляем веб страницу
}


void handleAbout() {
 webServer.send_P(200, "text/html", aboutHtml); // отправляем веб страницу
}


void handleCss() {
 webServer.send_P(200, "text/css", generalCss); // отправляем талицу стилей
}


void handleNotFound(){
  webServer.send(404, "text/plain", "404: Not found");
}


void handleMove(){
  String direction = webServer.arg("direction");  // получаем значение направления перемещения (была нажата или отдата кнопка движения)
  String value = webServer.arg("value");  // значение была ли нажата (value == 1) или отжата (value == 0) кнопка
  if (value.toInt() == 1){
    if (direction.equals("forward")) additionalAngle = moveAdditionalAngle;  
    else if (direction.equals("backward")) additionalAngle = -moveAdditionalAngle; 
    else if (direction.equals("left")) {
      additionalSpeed = rotateAdditionalSpeed;
      additionalAngle = moveAdditionalAngle; // немного наклоняемся вперед
    }
    else if (direction.equals("right")) {
      additionalSpeed = -rotateAdditionalSpeed;
      additionalAngle = moveAdditionalAngle; // немного наклоняемся вперед
    }
  } 
  else{
    additionalAngle = 0.f;
    additionalSpeed = 0.f;
  }
  webServer.send(200, "text/plane", "");
}


void handleGetData() {
 jsondoc["maxPvalue"] = MAX_P_VALUE;  // отправляем на страницу максимальные значения слайдеров настройки
 jsondoc["maxIvalue"] = MAX_I_VALUE;  // PID коэффициентов
 jsondoc["maxDvalue"] = MAX_D_VALUE;  //  
 jsondoc["kP"] = kP;                  // и текущие значения PID коэффициентов
 jsondoc["kI"] = kI;                  //
 jsondoc["kD"] = kD;                  //
 String output;
 serializeJson(jsondoc, output);      // создаем json-строку
 webServer.send(200, "application/json", output);   // отправляем ее на страницу
}


void handleSetPid() 
{
  kP = webServer.arg("kP").toFloat();   // получаем значения, введенные пользователем,  из на
  kI = webServer.arg("kI").toFloat();   //
  kD = webServer.arg("kD").toFloat();   //
  webServer.send(200, "text/plane", "");
}


void handleSaveData()
{
  StoredData stval; // создаем структуру, записываемую в eeprom
  stval.key = EEPROM_INIT_KEY;
  stval.kP = kP;
  stval.kI = kI;
  stval.kD = kD;
  EEPROM.put(EEPROM_RW_ADDR, stval);  // записываем 
  EEPROM.commit();
  webServer.send(200, "text/plane", "");
}


void moveA(int16_t speed){
  motors.setSpeedA(abs(speed));
  if (speed >= 0) motors.forwardA();
  else if (speed < 0) motors.backwardA();
}


void moveB(int16_t speed){
  motors.setSpeedB(abs(speed));
  if (speed >= 0) motors.forwardB();
  else if (speed < 0) motors.backwardB();
}


float pid(float input, float trgt, float kp, float ki, float kd, float dt, bool reset){
  static float integral = 0.f;
  static float lastError = 0.f; 

  if(reset){
    integral = 0.f;
    lastError = 0.f; 
    return 0.f;
  }
  float error = trgt - input;
  integral += error * dt * ki;
  float diff = (error - lastError) / dt;

  lastError = error;
  return error * kp + integral + diff * kd;
}


float getVoltage(){
  return (float)(constrain((int16_t)analogRead(VOLTAGE_PIN) - VOLTAGE_OFFSET, 0, 1024)) * VOLTAGE_REF * VOLTAGE_GAIN / 1024.f;
}


bool checkVoltage()
{
  static uint32_t checkTimer = millis();
  static bool result = true;  
  if (millis() > checkTimer + 1500) {   // проверяет напряжение раз в секунду
    result = getVoltage() > MIN_SUPPLY_VOLTAGE;
    checkTimer = millis();
  }
  return result;
}


void getAccGyro(float* accgyro){
  mpu.getMotion6(&ax, &ay, &az, &wx, &wy, &wz); // получаем данные с MPU
  ax += accXoffset;
  ay += accYoffset;
  az += accZoffset;
  wx += gyroXoffset;
  wy += gyroYoffset;
  wz += gyroZoffset;
  
  // преобразуем их
  accgyro[0] = ((float)ax) / 32768.f * 4.f; // ускорения
  accgyro[1] = ((float)ay) / 32768.f * 4.f;
  accgyro[2] = ((float)az) / 32768.f * 4.f;
  
  accgyro[3] = ((float)wx) / 32768.f * 500.f;  // угловые скорости
  accgyro[4] = ((float)wy) / 32768.f * 500.f;
  accgyro[5] = ((float)wz) / 32768.f * 500.f;
}


float getAngleY(){  // угол вокруг оси Y
  static uint32_t angleTimer = millis(); // локальный таймер
  static float angleY = 0.0;  // локально храним значения угла
  
  float accgyro[6];
  float dt;
    
  getAccGyro(accgyro);  // получаем преобразованные данные с IMU
  
  dt = ((float)(millis() - angleTimer) / 1000.f);  // промежуток времени между измерениями в секундах
          
  float accAngleY = atan2(accgyro[0], accgyro[2]);  // угол через ускорения = арктангенсу проекций на оси X и Z
  float gyroAngleY = angleY + DEG_TO_RAD*(-accgyro[4])*dt;   // угол через угловую скорость - интеграл скорости по оси Y
          
  angleY = alfaAg*accAngleY + (1-alfaAg)*gyroAngleY; // альфа-бета фильтрация
  angleTimer = millis();
      
  return angleY;
}


bool checkWallOnFront(){
  static bool isFrontWall = false;
  static uint32_t checkTimer = millis();
  if (millis() > checkTimer + 50) {   // проверяет расстояние раз в полсекунды
    isFrontWall = ultrasonicFront.read() <= minAfraidDistance;
    checkTimer = millis();
  }
  return isFrontWall;
}


bool checkWallOnBack(){
  static bool isBackWall = false;
  static uint32_t checkTimer = millis();
  if (millis() > checkTimer + 50) {   // проверяет расстояние раз в полсекунды
    isBackWall = ultrasonicBack.read() <= minAfraidDistance;
    checkTimer = millis();
  }
  return isBackWall;
}


void progressBar(uint8_t progress, String text){
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 10, "Loading: " + String(progress) + "%");
  display.drawProgressBar(3, 30, 125, 10, progress);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(5, 50, text);
  display.display();
}


void smoothProgressBar(uint8_t progressStart, uint8_t progressEnd, String text, uint8_t dt){
  for(uint8_t i = progressStart; i <= progressEnd; i++){  // плавная прорисовка
    progressBar(i, text);
    delay(dt);
  }
}


void displayLowVoltage(){
  display.clear();
  display.drawXbm(36, 20, BATTERY_WIDTH, BATTERY_HEIGHT, batteryBits); // рисуем 
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, "LOW VOLTAGE: " + String(getVoltage())+ " V");
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 50, "power on and reboot");
  display.display();
}


void displayFace(){
  static enum {
    HAPPY,
    NORMAL,
    CONFUSED,
    DISAPPOINTED
  } faceState = HAPPY;
  static uint32_t displayFaceTimer = millis();  // локальный таймер
  static uint32_t faceUpdatePerion = 700;       // период обновления морды робота в мс 
  static uint8_t count = 0;
  
  if (millis() > displayFaceTimer + faceUpdatePerion){
    switch(faceState){
      case HAPPY:{
        if (abs(angle) > stopAngle) faceState = CONFUSED;
        else if (abs(angle) > stopAngle/5) faceState = NORMAL;
        
        display.clear();
        display.drawXbm(24, 2, FACE_WIDTH, FACE_HEIGHT, faceHappyBits);  
        display.display();
        break;
      }
      case NORMAL:{
        if (abs(angle) > stopAngle) faceState = CONFUSED;
        else if (abs(angle) <= stopAngle/5) faceState = HAPPY;
        display.clear();
        display.drawXbm(24, 2, FACE_WIDTH, FACE_HEIGHT, faceNormalBits); 
        display.display();
        break;
      }
      case CONFUSED:{
        if (abs(angle) <= stopAngle/5) faceState = DISAPPOINTED; 
        display.clear();
        display.drawXbm(24, 2, FACE_WIDTH, FACE_HEIGHT, faceConfusedBits); 
        display.display();
        break;
      }
      case DISAPPOINTED:{
        if (count > 6) {
          faceState = HAPPY;
          count = 0;
          break;
        } else if (count > 3){
          display.clear();
          display.setFont(ArialMT_Plain_10);
          display.drawString(0, 0, "Hey meatbag! Couldn't \n you create me so \n that I don't fall?");
          display.display();
        } else {
          display.clear();
          display.drawXbm(24, 2, FACE_WIDTH, FACE_HEIGHT, faceDisappointedBits); 
          display.display();
        }
        count++;
        break;
      }
      default: {
          break;
      }
    }
    displayFaceTimer = millis();
  }
}


/*
void displayMpuRaw(){   // debug function
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "ax: " + String(ax));
  display.drawString(0, 20, "ay: " + String(ay));
  display.drawString(0, 40, "az: " + String(az));
  display.drawString(64, 0, "wx: " + String(wx));
  display.drawString(64, 20, "wy: " + String(wy));
  display.drawString(64, 40, "wz: " + String(wz));
  display.display();
}


void displayAngleAndSpeed(float speed, float angle){  // debug function
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 10, "angle: " + String((int)angle));
  display.drawString(0, 30, "speed: " + String((int)speed));
  display.display();
}


void displayDistance(float front, float back){
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 10, "distanceFront: " + String((int)front));
  display.drawString(0, 30, "distanceBack: " + String((int)back));
  display.display();
}
*/
