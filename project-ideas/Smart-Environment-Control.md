Dit is een slimme thermostaat, hier in zitten een aantal sensoren die de omgeving meten en de status laten zien op het scherm. Ook zitten er een paar knoppen op die dingen kunnen aanpassen. Er zit een microcontroller in die alle systemen bestuurd en het scherm aanstuurt. Het doel is om een thermostaat te maken die de atmosferische stand van de omgeving kan meten en laten zien.

![[Pasted image 20260210220316.png]]

## Ondersteuning

| requirement                                                                                                                 | bewijs                                                          |
| --------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------- |
| Watchdog timer                                                                                                              | het is een embedded systeem dus dat zit er in                   |
| Sleep modus                                                                                                                 | als de thermostaat niet aangeraakt wordt gaat het in sleep mode |
| Hardware interrupts                                                                                                         | als de knoppen gebruikt worden                                  |
| Software interrupts                                                                                                         | vindt plaats bij de sensoren en scherm                          |
| Gebruik van RTOS / embedded Linux                                                                                           | microcontroller werkt met OS                                    |
| Communicatie via I2C en/of SPI                                                                                              | scherm knoppen                                                  |
| Uitlezen van een sensor                                                                                                     | ja                                                              |
| Regelen van een systeem (bij voorkeur met PID-controller)                                                                   | de thermostaat                                                  |
| Aansturen van een actuator (motor driver, relais of iets dergelijks. Een LEDje is geen actuator.)                           | nee?                                                            |
| Interfacing met een display (LCD, OLED of anders; denk aan Nixie-tubes, meerdere LEDs, wijzertje) en knoppen of touchscreen | scherm                                                          |
## Mogelijk Materiaal Gebruik 
