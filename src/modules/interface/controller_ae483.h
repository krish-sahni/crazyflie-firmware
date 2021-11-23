#ifndef __CONTROLLER_AE483_H__
#define __CONTROLLER_AE483_H__

#include "stabilizer_types.h"

// An example struct to hold AE483-specific data sent from client to drone
struct AE483Data
{
  float x;
  float y;
  float z;
} __attribute__((packed));

void controllerAE483Init(void);
bool controllerAE483Test(void);
void controllerAE483(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

// Functions to receive measurements
void ae483UpdateWithTOF(tofMeasurement_t *tof);
void ae483UpdateWithFlow(flowMeasurement_t *flow);
void ae483UpdateWithDistance(distanceMeasurement_t *meas);

// Functions to receive AE483-specific data sent from client to drone
void ae483UpdateWithData(const struct AE483Data* data);


#endif //__CONTROLLER_AE483_H__
