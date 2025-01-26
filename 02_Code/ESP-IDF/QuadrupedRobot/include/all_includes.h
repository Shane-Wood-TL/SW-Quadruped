#ifndef __allIncludes__
#define __allIncludes__
#define map(a,b,c,w,q) ((w)+(((a) - (b)) * ((q) - (w)))/((c)-(b)))
#include <string>
#include <atomic>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/twai.h"
#include "driver/ledc.h"









#endif