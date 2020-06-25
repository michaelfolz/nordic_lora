# nordic_dogtracker
This will be a long term project, the overall goal of this project is to have a small cost effective device that will give approximate movement patterns and speed based on data collected from attached imu modules. 

	-Record IMU data at a rate of 133->333Hz, send out the imu data via BLE(if connected) otherwise save local EEPROM storage. 
	-Trigger a buzzer that emits sound and led when "local tracking" is designated (only when ble connected) and connection is out of range or no packets have been recieved after 3-5 seconds. (trigger bright led as well various patterns). - this should also be triggered by a custom button press. 
	

	-create a smart phone app or a separate embedded board 
		responsible for processing the IMU data into Acc/Velocity/distance
		triggering the alarm states, pairing and connecting via ble 
		displaying it on a easily readable format.      


Two main modes
//////////////
	- Active tracking, imu will be set to record and distribute data at 133Hz - 333Hz and processed on smart phone
	- Non Active tracking, imu will be set to store data at a rate of .2Hz - 1Hz, sleep modes will be utalized to promote battery life. 


Ideally the end device shall be small, battery life of at least 1 week w/o 

Tools & Devices Needed: 
  -NRF52840 Dongle - With Segger Jlink Programmer and Cable 

  
TODO:   - Too much to add. 
