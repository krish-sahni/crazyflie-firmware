#ifndef __CONTROLLER_AE483_H__
#define __CONTROLLER_AE483_H__

#include "stabilizer_types.h"

// typedef enum z_pos_mode_e {
//   zPosModeRawMocap    = 0,
//   zPosModeEncoded     = 1,
// } z_pos_mode_t;

// An example struct to hold AE483-specific data sent from client to drone
struct AE483Data
{

  float p_x;
  float p_y;
  float p_z;

} __attribute__((packed));

// struct AE483Data
// {
//   float p_x;
//   float p_y;
  
//   union
//   {
//     // Receive raw mocap data for z-position
//     float p_z;

//     // Receive encoded measurements of z-position
//     // and z-velocity.
//     struct
//     {
//       uint8_t qk_p_z;
//       uint8_t qk_v_z;
//     };
    
//   };
  
//   z_pos_mode_t zPosMode;
//   // float p_z;
//   // uint8_t qk_p_z;
//   // uint8_t qk_v_z;
// } __attribute__((packed));

void controllerAE483Init(void);
bool controllerAE483Test(void);
void controllerAE483(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

// Functions to receive measurements
void ae483UpdateWithTOF(tofMeasurement_t *tof);
void ae483UpdateWithFlow(flowMeasurement_t *flow);
void ae483UpdateWithDistance(distanceMeasurement_t *meas);
void ae483UpdateWithPosition(positionMeasurement_t *meas);
void ae483UpdateWithPose(poseMeasurement_t *meas);

// Functions to receive AE483-specific data sent from client to drone
void ae483UpdateWithData(const struct AE483Data* data);


#endif //__CONTROLLER_AE483_H__
