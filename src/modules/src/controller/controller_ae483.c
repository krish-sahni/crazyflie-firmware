#include "controller_ae483.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "stabilizer_types.h"
#include "power_distribution.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

#include "debug.h"

#define DEBUG_MODULE "CONTROLLER AE483"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;


// Sensor measurements
// - tof (from the z ranger on the flow deck)
static uint16_t tof_count = 0;
static float tof_distance = 0.0f;
// - flow (from the optical flow sensor on the flow deck)
static uint16_t flow_count = 0;
static float flow_dpixelx = 0.0f;
static float flow_dpixely = 0.0f;
// - Mocap data
static float p_x_mocap = 0.0f;
static float p_y_mocap = 0.0f;
static float p_z_mocap = 0.0f;
static float psi_mocap = 0.0f;
static float theta_mocap = 0.0f;
static float phi_mocap = 0.0f;
// - old mocap data
static float p_x_mocap_old = 0.0f;
static float p_y_mocap_old = 0.0f;
static float p_z_mocap_old = 0.0f;

// // Parameters
static bool use_observer = false;
static bool reset_observer = false;


// // State
// - Velocity thru finite differencing from mocap data
static float v_x_mocap = 0.0f;
static float v_y_mocap = 0.0f;
static float v_z_mocap = 0.0f;

// static float p_x = 0.0f;
// static float p_y = 0.0f;
// static float p_z = 0.0f;
// static float psi = 0.0f;
// static float theta = 0.0f;
// static float phi = 0.0f;
// static float v_x = 0.0f;
// static float v_y = 0.0f;
// static float v_z = 0.0f;
// static float w_x = 0.0f;
// static float w_y = 0.0f;
// static float w_z = 0.0f;

// // Setpoint
// static float p_x_des = 0.0f;
// static float p_y_des = 0.0f;
// static float p_z_des = 0.0f;

// // Input
// static float tau_x = 0.0f;
// static float tau_y = 0.0f;
// static float tau_z = 0.0f;
// static float f_z = 0.0f;

// // Motor power command
// static uint16_t m_1 = 0;
// static uint16_t m_2 = 0;
// static uint16_t m_3 = 0;
// static uint16_t m_4 = 0;

// // For time delay
// static float tau_x_cmd = 0.0f;    // tau_x command
// static float tau_y_cmd = 0.0f;    // tau_y command
// static float w_x_old = 0.0f;      // value of w_x from previous time step
// static float w_y_old = 0.0f;      // value of w_y from previous time step
// static float J_x = 1.68e-05f;     // FIXME: principal moment of inertia about x_B axis
// static float J_y = 1.69e-05;      // FIXME: principal moment of inertia about y_B axis
// static float dt = 0.002f;         // time step (corresponds to 500 Hz)

// // Measurements
// static float n_x = 0.0f;
// static float n_y = 0.0f;
// static float r = 0.0f;
// static float a_z = 0.0f;

// // Constants
// static float k_flow = 4.09255568f;
// static const float g = 9.81f;
// static const float m = 33e-3f;

// static float p_z_eq = 0.5f; // FIXME: replace with your choice of equilibrium height

// // Measurement errors
// static float n_x_err = 0.0f;
// static float n_y_err = 0.0f;
// static float r_err = 0.0f;

void ae483UpdateWithTOF(tofMeasurement_t *tof)
{
  tof_distance = tof->distance;
  tof_count++;
}

void ae483UpdateWithFlow(flowMeasurement_t *flow)
{
  flow_dpixelx = flow->dpixelx;
  flow_dpixely = flow->dpixely;
  flow_count++;
}

void ae483UpdateWithDistance(distanceMeasurement_t *meas)
{
  // If you have a loco positioning deck, this function will be called
  // each time a distance measurement is available. You will have to write
  // code to handle these measurements. These data are available:
  //
  //  meas->anchorId  uint8_t   id of anchor with respect to which distance was measured
  //  meas->x         float     x position of this anchor
  //  meas->y         float     y position of this anchor
  //  meas->z         float     z position of this anchor
  //  meas->distance  float     the measured distance
}

void ae483UpdateWithPosition(positionMeasurement_t *meas)
{
  // This function will be called each time you send an external position
  // measurement (x, y, z) from the client, e.g., from a motion capture system.
  // You will have to write code to handle these measurements. These data are
  // available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
}

void ae483UpdateWithPose(poseMeasurement_t *meas)
{
  // This function will be called each time you send an external "pose" measurement
  // (position as x, y, z and orientation as quaternion) from the client, e.g., from
  // a motion capture system. You will have to write code to handle these measurements.
  // These data are available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
  //  meas->quat.x    float     x component of quaternion from external orientation measurement
  //  meas->quat.y    float     y component of quaternion from external orientation measurement
  //  meas->quat.z    float     z component of quaternion from external orientation measurement
  //  meas->quat.w    float     w component of quaternion from external orientation measurement

  // Position
  static float px_temp;
  static float py_temp;
  static float pz_temp;

  px_temp = p_x_mocap;
  py_temp = p_y_mocap;
  pz_temp = p_z_mocap;

  p_x_mocap = meas->x;
  p_y_mocap = meas->y;
  p_z_mocap = meas->z;

  p_x_mocap_old = px_temp;
  p_y_mocap_old = py_temp;
  p_z_mocap_old = pz_temp;

  // Compute the velocity thru finite differencing
  v_x_mocap = (p_x_mocap - p_x_mocap_old) * 100.0f;
  v_y_mocap = (p_y_mocap - p_y_mocap_old) * 100.0f;
  v_z_mocap = (p_z_mocap - p_z_mocap_old) * 100.0f;

  // Orientation
  // - Create a quaternion from its parts
  struct quat q_mocap = mkquat(meas->quat.x, meas->quat.y, meas->quat.z, meas->quat.w);
  // - Convert the quaternion to a vector with yaw, pitch, and roll angles
  struct vec rpy_mocap = quat2rpy(q_mocap);
  // - Extract the yaw, pitch, and roll angles from the vector
  psi_mocap = rpy_mocap.z;
  theta_mocap = rpy_mocap.y;
  phi_mocap = rpy_mocap.x;

}

void ae483UpdateWithData(const struct AE483Data* data)
{
  // This function will be called each time AE483-specific data are sent
  // from the client to the drone. You will have to write code to handle
  // these data. For the example AE483Data struct, these data are:
  //
  //  data->x         float
  //  data->y         float
  //  data->z         float
  //
  // Exactly what "x", "y", and "z" mean in this context is up to you.
}


void controllerAE483Init(void)
{
  // Forward init calls to default PID controller
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
}

bool controllerAE483Test(void)
{
  // Forward call to attitude control test
  return attitudeControllerTest();
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void controllerAE483(control_t *control,
                     const setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlModeLegacy;

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {

    if (setpoint->mode.yaw == modeAbs) {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    } 

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, stabilizerStep)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);

    // float p_x_des;
    // float p_y_des;
    // float p_z_des;

    // p_x_des = setpoint->position.x;
    // p_y_des = setpoint->position.y;
    // p_z_des = setpoint->position.z;

    // float p_x;
    // float p_y;
    // float p_z;

    // p_x = state->position.x;
    // p_y = state->position.y;
    // p_z = state->position.z;

    // float v_x;
    // float v_y;
    // float v_z;

    // v_x = state->velocity.x;
    // v_y = state->velocity.y;
    // v_z = state->velocity.z;

    // attitudeDesired.roll = 0.0356779f*(p_y-p_y_des)+0.1223242f*v_y;
    // attitudeDesired.pitch = -0.0356779f*(p_x-p_x_des)-0.1223242f*v_x;
    // actuatorThrust = 1000.0f*(-200.00000f*(p_z-p_z_des)-0.2000000f*v_z) + 36000.0f;
    // if (actuatorThrust < 20000.0f)
    // {
    //   actuatorThrust = 20000.0f;
    // }
    // attitudeDesired.yaw = 0.0f;

  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {

      actuatorThrust = 0.0f;

    }

    // Attitude PID:
    // Input:  Desired attitude
    // Output: Desired attitude rates
    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);


    // Attitude rate PID:
    // Input:   Desired attitude rates
    // Output:  Roll/Pitch/Yaw body torques
    // Q). Where are the output variables from PID?
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    // Convert torques to actuator ouputs
    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
  }

  control->thrust = actuatorThrust;

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

//              1234567890123456789012345678 <-- max total length
//              group   .name
LOG_GROUP_START(ae483log)
LOG_ADD(LOG_UINT16,      num_tof,                &tof_count)
LOG_ADD(LOG_UINT16,      num_flow,               &flow_count)
LOG_ADD(LOG_FLOAT,       p_x_mocap,              &p_x_mocap)
LOG_ADD(LOG_FLOAT,       p_y_mocap,              &p_y_mocap)
LOG_ADD(LOG_FLOAT,       p_z_mocap,              &p_z_mocap)
LOG_ADD(LOG_FLOAT,       v_x_mocap,              &v_x_mocap)
LOG_ADD(LOG_FLOAT,       v_y_mocap,              &v_y_mocap)
LOG_ADD(LOG_FLOAT,       v_z_mocap,              &v_z_mocap)
LOG_ADD(LOG_FLOAT,       psi_mocap,              &psi_mocap)
LOG_ADD(LOG_FLOAT,       theta_mocap,            &theta_mocap)
LOG_ADD(LOG_FLOAT,       phi_mocap,              &phi_mocap)
// LOG_ADD(LOG_FLOAT,       p_x,                    &p_x)
// LOG_ADD(LOG_FLOAT,       p_y,                    &p_y)
// LOG_ADD(LOG_FLOAT,       p_z,                    &p_z)
// LOG_ADD(LOG_FLOAT,       psi,                    &psi)
// LOG_ADD(LOG_FLOAT,       theta,                  &theta)
// LOG_ADD(LOG_FLOAT,       phi,                    &phi)
// LOG_ADD(LOG_FLOAT,       v_x,                    &v_x)
// LOG_ADD(LOG_FLOAT,       v_y,                    &v_y)
// LOG_ADD(LOG_FLOAT,       v_z,                    &v_z)
// LOG_ADD(LOG_FLOAT,       w_x,                    &w_x)
// LOG_ADD(LOG_FLOAT,       w_y,                    &w_y)
// LOG_ADD(LOG_FLOAT,       w_z,                    &w_z)
// LOG_ADD(LOG_FLOAT,       p_x_des,                &p_x_des)
// LOG_ADD(LOG_FLOAT,       p_y_des,                &p_y_des)
// LOG_ADD(LOG_FLOAT,       p_z_des,                &p_z_des)
// LOG_ADD(LOG_FLOAT,       tau_x,                  &tau_x)
// LOG_ADD(LOG_FLOAT,       tau_y,                  &tau_y)
// LOG_ADD(LOG_FLOAT,       tau_z,                  &tau_z)
// LOG_ADD(LOG_FLOAT,       f_z,                    &f_z)
// LOG_ADD(LOG_UINT16,      m_1,                    &m_1)
// LOG_ADD(LOG_UINT16,      m_2,                    &m_2)
// LOG_ADD(LOG_UINT16,      m_3,                    &m_3)
// LOG_ADD(LOG_UINT16,      m_4,                    &m_4)
// LOG_ADD(LOG_FLOAT,       tau_x_cmd,              &tau_x_cmd)
// LOG_ADD(LOG_FLOAT,       tau_y_cmd,              &tau_y_cmd)
// LOG_ADD(LOG_FLOAT,       n_x,                    &n_x)
// LOG_ADD(LOG_FLOAT,       n_y,                    &n_y)
// LOG_ADD(LOG_FLOAT,       r,                      &r)
// LOG_ADD(LOG_FLOAT,       a_z,                    &a_z)
LOG_GROUP_STOP(ae483log)

//                1234567890123456789012345678 <-- max total length
//                group   .name
PARAM_GROUP_START(ae483par)
PARAM_ADD(PARAM_UINT8,     use_observer,            &use_observer)
PARAM_ADD(PARAM_UINT8,     reset_observer,          &reset_observer)
PARAM_GROUP_STOP(ae483par)
