# Esp32_Bme280_VoltageWatchdog
Small WebUI application to watch 4 voltage inputs on ADC1 from solar panel controller via resistive dividers.

WebUI servers an http://esp32.local/latest.csv file containing voltage data to be parsed by external/remote application 
for future developement (sending data via JS8CALL /LoRA or other radio transmission ways independend of Internet ???)
Also there is an bme280 sensor to read and log temperature/pressure/humidity served to latest.csv file and log to remote MySQL server
configurable in settings page.
At fresh start, esp32 advertise WifiAP "ESPstation" with pass "123456".
Access http://esp32.local/ to configure several AP in settings, then AutoAP will be disabled.
