<h1 align="center">
  <a href="https://youtu.be/ZRSPSNteWiU"><img src="https://github.com/TrashRobotics/BalancingRobot/blob/main/img/bbot.jpg" alt="Автоматическая неваляшка" width="800"></a>
  <br>
  Балансирующий робот на NodeMCU
  <br>
</h1>

<p align="center">
  <a href="https://github.com/TrashRobotics/BalancingRobot/blob/main/README.md">Русский</a> •
  <a href="https://github.com/TrashRobotics/BalancingRobot/blob/main/README-en.md">English(Английский)</a> 
</p>

# Описание проекта

Балансирующий робот, веселья ради, способен:     
* собственно, балансировать;  :)
* ездить и поворачивать;
* отклоняться от препятствий при помощи датчиков расстояния;
* все это непотребство управляется через точку доступа, раздаваемую 
  самим роботом, который выступает, в том числе, в качестве web-сервера,
  выдавая страницу управления в стандартный браузер, по запросу.
  
# Основные детали
* esp8266 NodeMCU v3;
* драйвер двигателей l298n;
* желтые одноосевые ардуино TT-мотор-редукторы;
* 2x 18650 Аккумуляторы и батарейный отсек для них
* MPU6050;
* SSD1306 128x64 I2C дисплей;
* 2x HC-SR04 датчики расстояния;
* тумблер KCD1-11;
* макетная плата 4x6 см (по желанию);
* [детали корпуса](https://www.thingiverse.com/thing:4967128)

![Основные детали](https://github.com/TrashRobotics/BalancingRobot/blob/main/img/parts.jpg)

# Схема подключения
![Схема подключения](https://github.com/TrashRobotics/BalancingRobot/blob/main/img/schematic.jpeg)

# Установка и прошивка
Для компиляции скетча нужно установить следующие библиотеки:
* [ArduinoJson](https://github.com/bblanchon/ArduinoJson);
* [L298N](https://github.com/AndreaLombardo/L298N);
* [MPU6050](https://github.com/ElectronicCats/mpu6050);
* [Ultrasonic](https://github.com/ErickSimoes/Ultrasonic);
* [ThingPulse SSD1306](https://github.com/ThingPulse/esp8266-oled-ssd1306);

Библиотеки устанавливаются переходом по ссылкам и нажатием **Code->Download ZIP**.            
В Arduino IDE **Скетч->Подключить библиотеку->Добавить .ZIP библиотеку...**

После этого скачиваем директорию balancing_robot и кладем ее в рабочее пространство Arduino IDE.
(sources.h должен лежать в одной папке со скетчем).

Компилируем и прошиваем.

# Включение и подключение
При включении питания робот начнет балансировать.    
Для управления необходимо подключиться к точке доступа:
```
ssid: balancingbot;   // по умолчанию
psw: 123456789;
```
и ввести в браузере ip-адрес робота:
```
192.168.1.1   // по умолчанию
```
развлекайтесь
