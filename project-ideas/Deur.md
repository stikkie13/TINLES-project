Deur die opent als er iets in de buurt is. Eigenlijk is het een automatische scharnier. Er komt boven de deur een sensor ding dat meet of er iemand in de buurt is en dan duwt een actuator de deur open. Het doel is om deur.

## Ondersteuning

| requirement                                                                                                                 | bewijs                                        |
| --------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------- |
| Watchdog timer                                                                                                              | het is een embedded systeem dus dat zit er in |
| Sleep modus                                                                                                                 | als sensor niks detecteerd                    |
| Hardware interrupts                                                                                                         | sensor?                                       |
| Software interrupts                                                                                                         | deur openen                                   |
| Gebruik van RTOS / embedded Linux                                                                                           | ja                                            |
| Communicatie via I2C en/of SPI                                                                                              | actuator                                      |
| Uitlezen van een sensor                                                                                                     | ja                                            |
| Regelen van een systeem (bij voorkeur met PID-controller)                                                                   | deur                                          |
| Aansturen van een actuator (motor driver, relais of iets dergelijks. Een LEDje is geen actuator.)                           | deur                                          |
| Interfacing met een display (LCD, OLED of anders; denk aan Nixie-tubes, meerdere LEDs, wijzertje) en knoppen of touchscreen | nixie-tubes                                   |
## Mogelijk Materiaal Gebruik 
deur