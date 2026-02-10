Dit is een air-fryer. Er zit een hitte element boven een bak, het hitte element word dan heet waardoor de lucht heet word. De bak kan uitgeschoven worden om er iets in te doen en dan dicht geschoven worden zodat de hete lucht niet ontsnapt. Het hitte element wordt aangestuurd door een microcontroller die ook het scherm aanstuurt. Op het scherm komt de status van de air-fryer weergeven. Het doel is om een air-fryer te maken die eten kan opwarmen in een uitschuifbare bak.

![[Pasted image 20260210220340.png]]

## Ondersteuning

| requirement                                                                                                                 | bewijs                                            |
| --------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------- |
| Watchdog timer                                                                                                              | het is een embedded systeem dus dat zit er in     |
| Sleep modus                                                                                                                 | bak lang dicht zonder aanzetten van gebruiker     |
| Hardware interrupts                                                                                                         | bak dicht/open                                    |
| Software interrupts                                                                                                         | timer voor hoe lang de air-fryer aan moet blijven |
| Gebruik van RTOS / embedded Linux                                                                                           | ja                                                |
| Communicatie via I2C en/of SPI                                                                                              | sensoren, scherm                                  |
| Uitlezen van een sensor                                                                                                     | hitte meter en of de bak dicht is                 |
| Regelen van een systeem (bij voorkeur met PID-controller)                                                                   | hitte element                                     |
| Aansturen van een actuator (motor driver, relais of iets dergelijks. Een LEDje is geen actuator.)                           | hitte element                                     |
| Interfacing met een display (LCD, OLED of anders; denk aan Nixie-tubes, meerdere LEDs, wijzertje) en knoppen of touchscreen | scherm                                            |
## Mogelijk Materiaal Gebruik 
