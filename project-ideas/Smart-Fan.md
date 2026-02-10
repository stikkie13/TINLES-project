Dit is een fan die slim beweegt gebaseerd op de lokale atmosfeer. De fan draait naar een bepaalde richting gebaseerd op atmosferische data die wordt gemeten door sensoren in de fan. Ook zit er een knop op die de fan in harde of zachte modus kan zetten, of uit. Er zitten een aantal motoren in de fan die de rotatie van de blades en de richting van de lucht stroom besturen. De motoren worden aangestuurd door een microcontroller, de status van de fan wordt laten zien op een scherm. Het doel is om een zelf draaiende fan te maken die gebaseerd op de omgeving de juiste kant op draait.

![[Pasted image 20260210220246.png]]
## Ondersteuning

| Requirement                                                                                                                 | Bewijs                                                                                            |
| --------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| Watchdog timer                                                                                                              | het is een embedded systeem dus dat zit er in                                                     |
| Sleep modus                                                                                                                 | Als de de fan uit wordt gezet maar wel in het stopcontact blijft zitten gaat de fan in sleep mode |
| Hardware interrupts                                                                                                         | als de fan in een harde/zachte modus word gezet kan er een hardware interrupt plaats vinden       |
| Software interrupts                                                                                                         | software interrupts gebeuren als het scherm geüpdate moet worden                                  |
| Gebruik van RTOS / embedded Linux                                                                                           | de microcontroller werkt op een RTOS / embedded Linux besturingssysteem                           |
| Communicatie via I2C en/of SPI                                                                                              | de motoren en het scherm wordt mee gecommuniceerd                                                 |
| Uitlezen van een sensor                                                                                                     | voor atmosferische data worden sensoren uitgelezen                                                |
| Regelen van een systeem (bij voorkeur met PID-controller)                                                                   | voor het bewegen van de richting van de fan wordt een systeem gebruikt                            |
| Aansturen van een actuator (motor driver, relais of iets dergelijks. Een LEDje is geen actuator.)                           | motoren 🖕                                                                                        |
| Interfacing met een display (LCD, OLED of anders; denk aan Nixie-tubes, meerdere LEDs, wijzertje) en knoppen of touchscreen | scherm                                                                                            |
## Mogelijk Materiaal Gebruik 
