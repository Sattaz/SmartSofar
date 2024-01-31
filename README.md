# SmartSofar
Web interface to control multiple Sofar ME3000 battery chargers

Project based on the code from mister cmcgerty : https://github.com/cmcgerty/Sofar2mqtt

Integrated multi-registers read by code from : https://github.com/irekzielinski

Thanks a lot to them for their support!


The electronic circuit to be realized is exactly the same as the one described by mister cmcgerty on his GitHub.


__Version 1.0 (Jan-2024)__  
* Multi-Sofar chargers handling
* Shelly 3M Pro Energy Meter integration, used in automated mode to adjust the power
* Reads all Sofar registers in one go making read cycle very quick (thx to Irek for sharing his solution!)
* Asynchronous Web interface to display live data and control the chargers
* Automatic / Manual mode control switch from the Web interface
* Multiple chargers power commands will be set equaliy to each charger (ex: if 3000W is set to charge and 2 chargers are connected then each will charge at 1500W)
* Adjust power (Charge/Discharge) by slider and Standby on the Web interface in Manual mode
* Retains last enabled mode in esp eprom
* Live ESP logs accessible from the Web interface
* JSON data accessible via CURL:  
  http://xxx.xxx.xxx.xxx/data.json
* CURL to set mode, standby/charge/discharge commands:  
  http://xxx.xxx.xxx.xxx/update?cmd=setmode&val=x (x=0 for Mode Manu, x=1 for Mode Auto)  
  http://xxx.xxx.xxx.xxx/update?cmd=setpower&val=x (x=0 to set Standby, x>0 to charge to xW, x<0 to discharge to xW)
* Comparison of x numbers of EM Shelly measurements (to be defined in the parameters) before placing an automatic adjustment command when the measured power is below a threshold (also to be defined in the parameters)  
  This because with low power the Shelly Energy Meter is not the most precise.  
  If the difference between the x numbers of concecutive power measurements is below a threshold (also to be defined) then the system adjusts.  
  With a power measurement greater than the defined threshold, the system adjusts from the first measurement.
* NOT YET implemented : initial settings from SD-CARD and adjusting parameter settings from the Web interface
* NOT YET implemented : MQTT Server (maybe using TinyMqtt) to host data and set charge/discharge/standby commands.


Any help is welcome to continue the development !

<p align="center" width="100%"><img src="https://github.com/Sattaz/SmartSofar/blob/main/Pictures/SmartSofar.jpg"></p>
