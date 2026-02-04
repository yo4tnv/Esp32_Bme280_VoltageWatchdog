# Esp32_Bme280_VoltageWatchdog
Small WebUI application to watch 4 voltage inputs on ADC1 from solar panel controller via resistive dividers.

This esp32 project ensure surveillance of voltages present on inputs and output of this automatic source switching device: 
https://www.aliexpress.com/item/1005007572764163.html. One input feeds 12-14v from solar panels charge controller,
the other input takes 12v of an ACmains switching power supply. When solar power goes off (controller sense battery too
low) this device switches to other power supply.

WebUI servers an http://esp32.local/latest.csv file containing voltage data to be parsed by external/remote application 
for future developement (??? sending data via JS8CALL/LoRA/APRS or other radio transmission means, independent of Internet ???)
Also there is an bme280 sensor to read and log temperature/pressure/humidity served to latest.csv file and log to remote MySQL server
configurable in settings page.

After compiling and uploading ino file at fresh start, esp32 advertise WifiAP "ESPstation" with pass "123456".
Access http://esp32.local/ to configure several AP in settings, then AutoAP will be disabled.
