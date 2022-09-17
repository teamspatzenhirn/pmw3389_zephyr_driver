//
// Created by jonasotto on 9/15/22.
//

#ifndef CAROLO_APP_PMW3389_H
#define CAROLO_APP_PMW3389_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/sensor.h>

enum sensor_channel_pmw3389 {
	SENSOR_CHAN_PMW3389_DISTANCE_X = SENSOR_CHAN_PRIV_START,
	SENSOR_CHAN_PMW3389_DISTANCE_Y
};

#ifdef __cplusplus
}
#endif
#endif // CAROLO_APP_PMW3389_H
