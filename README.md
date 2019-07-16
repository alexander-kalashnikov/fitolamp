It is a firmware for fitolamp project placed on the EasyEDA: https://easyeda.com/kalashnikov.alexander.b/fitolamp

It can be used as a smart fitolamp that controls luminosity and power on time to extend daylight hours for some herbs/plants.

There are two sensors: movement sensor (connected to PB8 STM32 pin) and I2C luxmeter (GY-302).
Movement sensor used as a trigger to dim down the luminosity if someone is near the lamp.
Luxmeter is used to hold the luminosity at some level. It helps to dim LEDs if there are enough light already, for example.

Additional UART connector can be used for UART or bluetooth connection: for date and time configuration, or a schedule changing, or anything else.
