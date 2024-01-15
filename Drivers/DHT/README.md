# DHT Library for STM32 HAL
[![ko-fi](https://www.ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/O5O4221XY)
* http://www.github.com/NimaLTD   
* https://www.instagram.com/github.nimaltd/   
* https://www.youtube.com/@nimaltd  

This is the DHT (Humidity-Temperature) sensors STM32 HAL Library  

How to use this Library:
* Select "General peripheral Initalizion as a pair of '.c/.h' file per peripheral" on project settings.   
* Enable a gpio pin as external interrupt on both Rising and Falling edge.
* Enable a timer as normal mode.   
* Include Header and source into your project.   
* Config "dhtConf.h".   
* Create your DHT_t struct.   
* Put DHT_pinChangeCallBack(your struct) in external callback function.   
* Call DHT_init( .. .. .. ).   
* Read data by DHT_readData(&temp,&humid)

```
#include "DHT.h"
DHT_t     am2301;
bool      am2301_ok;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == am2301.pin)
  {
    DHT_pinChangeCallBack(&am2301);
  }
}

int main()
{
  DHT_init(&am2301,DHT_Type_AM2301,&htim7,72,AM2301_GPIO_Port,AM2301_Pin); 
  while(1)
  {
    HAL_Delay(5000);
    float t,h;
    am2301_ok = DHT_readData(&am2301,&t,&h);  
  }
}

```
<a ><img src="1.jpg" height="300"/></a>
<a ><img src="2.jpg" height="300"/></a>

Reversion History:   
(2.0.0)   
rewrite completly and Add external interrupt to reading pulses.   
  
(1.0.0)   
First Release.   


