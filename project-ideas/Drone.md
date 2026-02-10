Dit is een drone die kan vliegen met behulp van propellers. Het is een systeem van 4 motoren die worden aangestuurd via een microcontroller en een aantal sensoren. De sensoren geven de stand van de drone aan en geven dat door aan de microcontroller die de propellers aanstuurt om stabiel in de lucht te blijven. Het doel is om een drone in de lucht te krijgen en deze ook stabiel te laten zweven.

![[Pasted image 20260210220328.png]]

## Ondersteuning

| requirement                                                                                                                 | bewijs                                               |
| --------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------- |
| Watchdog timer                                                                                                              | het is een embedded systeem dus dat zit er in        |
| Sleep modus                                                                                                                 | tis een drone, die shit moet aan blijven in de lucht |
| Hardware interrupts                                                                                                         | vast wel                                             |
| Software interrupts                                                                                                         | ja                                                   |
| Gebruik van RTOS / embedded Linux                                                                                           | sure                                                 |
| Communicatie via I2C en/of SPI                                                                                              | motoren                                              |
| Uitlezen van een sensor                                                                                                     | wat denk je zelf                                     |
| Regelen van een systeem (bij voorkeur met PID-controller)                                                                   | i dont careeeeee                                     |
| Aansturen van een actuator (motor driver, relais of iets dergelijks. Een LEDje is geen actuator.)                           | vast wel                                             |
| Interfacing met een display (LCD, OLED of anders; denk aan Nixie-tubes, meerdere LEDs, wijzertje) en knoppen of touchscreen | kill me                                              |
## Mogelijk Materiaal Gebruik 
