#include "controller_ae483.h"
#include "stabilizer_types.h"
#include "power_distribution.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

// Sensor measurements
// - tof (from the z ranger on the flow deck)
static uint16_t tof_count = 0;
static float tof_distance = 0.0f;
// - flow (from the optical flow sensor on the flow deck)
static uint16_t flow_count = 0;
static float flow_dpixelx = 0.0f;
static float flow_dpixely = 0.0f;

// - Mocap measurements
static float p_x_mocap = 0.0f;
static float p_y_mocap = 0.0f;
static float p_z_mocap = 0.0f;
static float psi_mocap = 0.0f;
static float theta_mocap = 0.0f;
static float phi_mocap = 0.0f;

//new log testing
static uint8_t log_x;
static uint8_t log_y;
static uint8_t log_z;

// Parameters
static bool use_observer = false;
static bool reset_observer = false;


// State
static float p_x = 0.0f;
static float p_y = 0.0f;
static float p_z = 0.0f;
static float psi = 0.0f;
static float theta = 0.0f;
static float phi = 0.0f;
static float v_x = 0.0f;
static float v_y = 0.0f;
static float v_z = 0.0f;
static float w_x = 0.0f;
static float w_y = 0.0f;
static float w_z = 0.0f;

// Setpoint
static float p_x_des = 0.0f;
static float p_y_des = 0.0f;
static float p_z_des = 0.0f;

// Input
static float tau_x = 0.0f;
static float tau_y = 0.0f;
static float tau_z = 0.0f;
static float f_z = 0.0f;

// Motor power command
static uint16_t m_1 = 0;
static uint16_t m_2 = 0;
static uint16_t m_3 = 0;
static uint16_t m_4 = 0;

// For time delay
static float tau_x_cmd = 0.0f;    // tau_x command
static float tau_y_cmd = 0.0f;    // tau_y command
static float w_x_old = 0.0f;      // value of w_x from previous time step
static float w_y_old = 0.0f;      // value of w_y from previous time step
static float J_x = 1.68e-05f;     // FIXME: principal moment of inertia about x_B axis
static float J_y = 1.69e-05;      // FIXME: principal moment of inertia about y_B axis
static float dt = 0.002f;         // time step (corresponds to 500 Hz)

// Measurements
static float n_x = 0.0f;
static float n_y = 0.0f;
static float r = 0.0f;
static float a_z = 0.0f;

// Constants
static float k_flow = 4.09255568f;
static float g = 9.81f;
static float p_z_eq = 0.5f; // FIXME: replace with your choice of equilibrium height

// Measurement errors
static float n_x_err = 0.0f;
static float n_y_err = 0.0f;
static float r_err = 0.0f;

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
  log_x = meas->x;
  log_y = meas->y;
  log_z = meas->z;
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
  p_x_mocap = meas->x;
  p_y_mocap = meas->y;
  p_z_mocap = meas->z;

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
  log_x = data->x;
  log_y = data->y;
  log_z = data->z;
}


void controllerAE483Init(void)
{
  // Do nothing
}

bool controllerAE483Test(void)
{
  // Do nothing (test is always passed)
  return true;
}

void controllerAE483(control_t *control,
                     const setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const stabilizerStep_t stabilizerStep)
{
  // Make sure this function only runs at 500 Hz
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    return;
  }

  // Desired position
  p_x_des = setpoint->position.x;
  p_y_des = setpoint->position.y;
  p_z_des = setpoint->position.z;

  // Measurements
  w_x = radians(sensors->gyro.x);
  w_y = radians(sensors->gyro.y);
  w_z = radians(sensors->gyro.z);
  a_z = g * sensors->acc.z;
  n_x = flow_dpixelx;
  n_y = flow_dpixely;
  r = tof_distance;

  // Torques by finite difference
  tau_x = J_x * (w_x - w_x_old) / dt;
  tau_y = J_y * (w_y - w_y_old) / dt;
  w_x_old = w_x;
  w_y_old = w_y;

  if (reset_observer) {
    p_x = 0.0f;
    p_y = 0.0f;
    p_z = 0.0f;
    psi = 0.0f;
    theta = 0.0f;
    phi = 0.0f;
    v_x = 0.0f;
    v_y = 0.0f;
    v_z = 0.0f;
    reset_observer = false;
  }

  // State estimates
  if (use_observer) {
  
    // Compute each element of:
    // 
    //   C x + D u - y
    // 
    // FIXME: your code goes here
    n_x_err = -k_flow * w_y + k_flow * v_x / p_z_eq - n_x;
    n_y_err = -k_flow * w_x + k_flow * v_y / p_z_eq - n_y;
    r_err = p_z - r;


    // Update estimates
    // FIXME: your code goes here
    p_x += dt * v_x;  
    p_y += dt * v_y;  
    p_z += dt * (v_z - 21.781746f * r_err);
    psi += dt * w_z;
    theta += dt * (w_y - 0.001497f * n_x_err);
    phi += dt * (w_x + 0.001652f * n_y_err);
    v_x += dt * (g * theta - 0.061758f * n_x_err);
    v_y += dt * (-g * phi - 0.064906f * n_y_err);
    v_z += dt * (a_z - g - 98.333333f * r_err);

  } else {
    p_x = state->position.x;
    p_y = state->position.y;
    p_z = state->position.z;
    psi = radians(state->attitude.yaw);
    theta = - radians(state->attitude.pitch);
    phi = radians(state->attitude.roll);
    v_x = state->velocity.x*cosf(psi)*cosf(theta) + state->velocity.y*sinf(psi)*cosf(theta) - state->velocity.z*sinf(theta);
    v_y = state->velocity.x*(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)) + state->velocity.y*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)) + state->velocity.z*sinf(phi)*cosf(theta);
    v_z = state->velocity.x*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)) + state->velocity.y*(-sinf(phi)*cosf(psi) + sinf(psi)*sinf(theta)*cosf(phi)) + state->velocity.z*cosf(phi)*cosf(theta);
  }

  if (setpoint->mode.z == modeDisable) {
    // If there is no desired position, then all
    // motor power commands should be zero

    m_1 = 0;
    m_2 = 0;
    m_3 = 0;
    m_4 = 0;
  } else {
    // Otherwise, motor power commands should be
    // chosen by the controller

    // FIXME (CONTROLLER GOES HERE)
    tau_x_cmd = 0.00657196f * (p_y - p_y_des) -0.01089931f * phi + 0.00394655f * v_y -0.00149955f * w_x -2.74590184f * tau_x;
    tau_y_cmd = -0.00657196f * (p_x - p_x_des) -0.01090571f * theta -0.00394763f * v_x -0.00150133f * w_y -2.73765352f * tau_y;
    tau_z = -0.00096575f * psi -0.00038228f * w_z;
    f_z = -0.34995563f * (p_z - p_z_des) -0.16359250f * v_z + 0.33648300f;

    // FIXME (METHOD OF POWER DISTRIBUTION GOES HERE)
    m_1 = limitUint16( -2937720.3f * tau_x_cmd -2937720.3f * tau_y_cmd -34626038.8f * tau_z + 135135.1f * f_z );
    m_2 = limitUint16( -2937720.3f * tau_x_cmd + 2937720.3f * tau_y_cmd + 34626038.8f * tau_z + 135135.1f * f_z );
    m_3 = limitUint16( 2937720.3f * tau_x_cmd + 2937720.3f * tau_y_cmd -34626038.8f * tau_z + 135135.1f * f_z );
    m_4 = limitUint16( 2937720.3f * tau_x_cmd -2937720.3f * tau_y_cmd + 34626038.8f * tau_z + 135135.1f * f_z );
  }

  // Apply motor power commands
  control->m1 = m_1;
  control->m2 = m_2;
  control->m3 = m_3;
  control->m4 = m_4;

}

//              1234567890123456789012345678 <-- max total length
//              group   .name
LOG_GROUP_START(ae483log)
LOG_ADD(LOG_UINT16,      num_tof,                &tof_count)
LOG_ADD(LOG_UINT16,      num_flow,               &flow_count)
LOG_ADD(LOG_FLOAT,       p_x,                    &p_x)
LOG_ADD(LOG_FLOAT,       p_y,                    &p_y)
LOG_ADD(LOG_FLOAT,       p_z,                    &p_z)
LOG_ADD(LOG_FLOAT,       psi,                    &psi)
LOG_ADD(LOG_FLOAT,       theta,                  &theta)
LOG_ADD(LOG_FLOAT,       phi,                    &phi)
LOG_ADD(LOG_FLOAT,       v_x,                    &v_x)
LOG_ADD(LOG_FLOAT,       v_y,                    &v_y)
LOG_ADD(LOG_FLOAT,       v_z,                    &v_z)
LOG_ADD(LOG_FLOAT,       w_x,                    &w_x)
LOG_ADD(LOG_FLOAT,       w_y,                    &w_y)
LOG_ADD(LOG_FLOAT,       w_z,                    &w_z)
LOG_ADD(LOG_FLOAT,       p_x_des,                &p_x_des)
LOG_ADD(LOG_FLOAT,       p_y_des,                &p_y_des)
LOG_ADD(LOG_FLOAT,       p_z_des,                &p_z_des)
LOG_ADD(LOG_FLOAT,       tau_x,                  &tau_x)
LOG_ADD(LOG_FLOAT,       tau_y,                  &tau_y)
LOG_ADD(LOG_FLOAT,       tau_z,                  &tau_z)
LOG_ADD(LOG_FLOAT,       f_z,                    &f_z)
LOG_ADD(LOG_UINT16,      m_1,                    &m_1)
LOG_ADD(LOG_UINT16,      m_2,                    &m_2)
LOG_ADD(LOG_UINT16,      m_3,                    &m_3)
LOG_ADD(LOG_UINT16,      m_4,                    &m_4)
LOG_ADD(LOG_FLOAT,       tau_x_cmd,              &tau_x_cmd)
LOG_ADD(LOG_FLOAT,       tau_y_cmd,              &tau_y_cmd)
LOG_ADD(LOG_FLOAT,       n_x,                    &n_x)
LOG_ADD(LOG_FLOAT,       n_y,                    &n_y)
LOG_ADD(LOG_FLOAT,       r,                      &r)
LOG_ADD(LOG_FLOAT,       a_z,                    &a_z)
LOG_ADD(LOG_FLOAT,       p_x_mocap,              &p_x_mocap)
LOG_ADD(LOG_FLOAT,       p_y_mocap,              &p_y_mocap)
LOG_ADD(LOG_FLOAT,       p_z_mocap,              &p_z_mocap)
LOG_ADD(LOG_FLOAT,       psi_mocap,              &psi_mocap)
LOG_ADD(LOG_FLOAT,       theta_mocap,            &theta_mocap)
LOG_ADD(LOG_FLOAT,       phi_mocap,              &phi_mocap)
LOG_ADD(LOG_UINT8,       foo,                    &log_x)
LOG_ADD(LOG_UINT8,       bar,                    &log_y)
LOG_ADD(LOG_UINT8,       baz,                    &log_z) 
LOG_GROUP_STOP(ae483log)

//                1234567890123456789012345678 <-- max total length
//                group   .name
PARAM_GROUP_START(ae483par)
PARAM_ADD(PARAM_UINT8,     use_observer,            &use_observer)
PARAM_ADD(PARAM_UINT8,     reset_observer,          &reset_observer)
PARAM_GROUP_STOP(ae483par)
