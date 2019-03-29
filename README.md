Grove - I2C Thermocouple Amplifier(MCP9600)
==============


Introduction of sensor
----------------------------  
Microchip Technology Inc.’s MCP9600 digital temperature sensor converts thermocouple EMF to degree Celsius with inetgrated cold-junction compensation.This device corrects the thermocouplenonlinear error characteristics of eight thermocouple types and outputs ±1.5℃ accurate temperature data for the selected thermocouple. 

***
Usage:
==========
Download all the source files.
There are two examples for user to use.
>* **example/MCP9600_basic_demo/MCP9600_basic_demo.ino**:This example is a sample use of temperature sensor,you need to polling for data. 
>* **example/MCP9600_4channel_INT_demo/MCP9600_4channel_INT_demo.ino**,There are four alert pads on the sensor module which connect to alert pin.You can set temperature limits by calling the API we provided.The alert pin outputs low when the temperature value beyond limit.You can attach alert pin to a interrupt pin of host,To improve the efficiency of program operation.

Reference:
===============
Refer to the **datasheet** to get more detail.



***
This software is written by downey  for seeed studio<br>
Email:dao.huang@seeed.cc
and is licensed under [The MIT License](http://opensource.org/licenses/mit-license.php). Check License.txt for more information.<br>

Contributing to this software is warmly welcomed. You can do this basically by<br>
[forking](https://help.github.com/articles/fork-a-repo), committing modifications and then [pulling requests](https://help.github.com/articles/using-pull-requests) (follow the links above<br>
for operating guide). Adding change log and your contact into file header is encouraged.<br>
Thanks for your contribution.

Seeed Studio is an open hardware facilitation company based in Shenzhen, China. <br>
Benefiting from local manufacture power and convenient global logistic system, <br>
we integrate resources to serve new era of innovation. Seeed also works with <br>
global distributors and partners to push open hardware movement.<br>
