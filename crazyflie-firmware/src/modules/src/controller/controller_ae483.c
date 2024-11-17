#include "controller_ae483.h"
#include "stabilizer_types.h"
#include "power_distribution.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "math.h"
#include "user_specific_logic.h"

// Sensor measurements
// - tof (from the z ranger on the flow deck)
static uint16_t tof_count = 0;
static float tof_distance = 0.0f;

// - flow (from the optical flow sensor on the flow deck)
static uint16_t flow_count = 0;
static float flow_dpixelx = 0.0f;
static float flow_dpixely = 0.0f;

// Parameters
static bool use_observer = false;
static bool reset = true; // Any reason not to default this to true?

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
static float psi_des = 0.0f;

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

// Measurements
static float n_x = 0.0f;
static float n_y = 0.0f;
static float r = 0.0f;
static float a_z = 9.81f;

// Constants
static float k_flow = K_FLOW;
static float g = 9.81f;
static float p_z_eq = P_Z_EQ; // FIXME: replace with your choice of equilibrium height

// Measurement errors
static float n_x_err = 0.0f;
static float n_y_err = 0.0f;
static float r_err = 0.0f;
static float err_0 = 0.0f;
static float err_1 = 0.0f;
static float err_2 = 0.0f;
static float err_3 = 0.0f;
static float err_4 = 0.0f;
static float err_5 = 0.0f;
static float err_6 = 0.0f;
static float err_7 = 0.0f;
static float err_8 = 0.0f;

// For time delay
static float tau_x_cmd = 0.0f;    // tau_x command
static float tau_y_cmd = 0.0f;    // tau_y command
static float w_x_old = 0.0f;      // value of w_x from previous time step
static float w_y_old = 0.0f;      // value of w_y from previous time step
static float J_x = J_X;          // FIXME: principal moment of inertia about x_B axis
static float J_y = J_Y;          // FIXME: principal moment of inertia about y_B axis
static float dt = 0.002f;         // time step (corresponds to 500 Hz)
static float a_z_0 = 9.81f;
static float d_carpet = 0.0497f;
static float d1d2 =  D1D2_INIT;

// Mocap
static uint8_t use_mocap = 0;
static float p_x_mocap = 0.0f;
static float p_y_mocap = 0.0f;
static float p_z_mocap = 0.0f;
static float psi_mocap = 0.0f;
static float theta_mocap = 0.0f;
static float phi_mocap = 0.0f;

// LED Deck
static uint8_t use_LED = 1;

// Safety variables
static uint8_t safe_to_set_motors = 1; // Safety variable! False if drone is unsafe.
static uint8_t use_safety = 1; // Use the safety checking system

// Initialize default_bounds array with hardcoded limits
#define NUM_BOUNDS 24
static float default_bounds[NUM_BOUNDS][2] = {
    {-80.0f, 80.0f}, // n_x bounds (N_X_LOWER, N_X_UPPER)
    {-80.0f, 80.0f}, // n_y bounds (N_Y_LOWER, N_Y_UPPER)
    {0.0f, 1.0f},    // r bounds (R_LOWER, R_UPPER)
    {-1.0f, 1.0f},   // psi bounds (PSI_LOWER, PSI_UPPER)
    {-0.2f, 0.2f},   // theta bounds (THETA_LOWER, THETA_UPPER)
    {-0.2f, 0.2f},   // phi bounds (PHI_LOWER, PHI_UPPER)
    {-1.0f, 1.0f},   // w_x bounds (W_X_LOWER, W_X_UPPER)
    {-1.0f, 1.0f},   // w_y bounds (W_Y_LOWER, W_Y_UPPER)
    {-1.5f, 1.5f},   // w_z bounds (W_Z_LOWER, W_Z_UPPER)
    {-1.0f, 1.0f},   // p_x dynamic bounds
    {-1.0f, 1.0f},   // p_y dynamic bounds
    {0.0f, 1.5f},    // p_z bounds (P_Z_LOWER, P_Z_UPPER)
    {-1.0f, 1.0f},   // v_x bounds (V_X_LOWER, V_X_UPPER)
    {-1.0f, 1.0f},   // v_y bounds (V_Y_LOWER, V_Y_UPPER)
    {-0.5f, 0.75f},  // v_z bounds (V_Z_LOWER, V_Z_UPPER)
    {-1.5f, 1.5f},   // p_x_int dynamic bounds
    {-1.5f, 1.5f},   // p_y_int dynamic bounds
    {-0.1f, 1.5f},    // p_z_int dynamic bounds (P_Z_INT_LOWER, P_Z_INT_UPPER)
    {-1.5f, 1.5f},   // v_x_int bounds (V_X_INT_LOWER, V_X_INT_UPPER)
    {-1.5f, 1.5f},   // v_y_int bounds (V_Y_INT_LOWER, V_Y_INT_UPPER)
    {-1.0f, 1.0f},   // v_z_int bounds (V_Z_INT_LOWER, V_Z_INT_UPPER)
    {-4.0f, 4.0f},   // a_x_in_W bounds
    {-4.0f, 4.0f},   // a_y_in_W bounds
    {4.0f, 18.0f}    // a_z_in_W bounds
};

// Declare and initialize bounds
static float bounds[NUM_BOUNDS][2];

static int8_t bounds_violation_index = -1;
static float bounds_violation_value = 0.0;
static uint8_t bounds_update_index = 63;
static float bounds_update_value = 0.0;
static uint32_t bounds_update = 4227858432;

// Backup state estimation variables
static float a_x_in_W = 0.0f;
static float a_y_in_W = 0.0f;
static float a_z_in_W = 0.0f;

// New measurments
static float a_x = 0.0f;
static float a_y = 0.0f;
static float a_x_0 = 0.0f;
static float a_y_0 = 0.0f;
static float v_x_int = 0.0f;
static float v_y_int = 0.0f;
static float v_z_int = 0.0f;
static float p_x_int = 0.0f;
static float p_y_int = 0.0f;
static float p_z_int = 0.0f;

// Flow ToF Age 
static uint32_t flow_age = 0;
static uint32_t r_age = 0;
static uint32_t mocap_age = 0;
static float flow_old = 0.0f;
static float r_old = 0.0f;
static float mocap_old = 0.0;
static uint32_t sample_count = 1;

// Debug variables starts
// Debug variables end

void initialize_observers(){
  // Using Qualisis Frame
  if (use_mocap){
    // Reset States
    p_x = p_x_mocap;
    p_y = p_y_mocap;
    p_z = p_z_mocap;
    psi = psi_mocap;
    theta = theta_mocap;
    phi = phi_mocap;
    v_x = 0.0f;
    v_y = 0.0f;
    v_z = 0.0f;
    // a_z_0 = a_z;  

    // Reset backup state estimation
    v_x_int = 0.0f;
    v_y_int = 0.0f;
    v_z_int = 0.0f;
    p_x_int = p_x_mocap;
    p_y_int = p_y_mocap;
    p_z_int = p_z_mocap;
    a_x_in_W = 0.0f;
    a_y_in_W = 0.0f;
    a_z_in_W = 0.0f;

    // Debug
    // Debug

    d_carpet = p_z_mocap - d1d2;
  } else {
    p_x = 0.0f;
    p_y = 0.0f;
    p_z = 0.0f;
    psi = 0.0f;
    theta = 0.0f;
    phi = 0.0f;
    v_x = 0.0f;
    v_y = 0.0f;
    v_z = 0.0f;
    // a_z_0 = a_z;  

    // Reset backup state estimation
    v_x_int = 0.0f;
    v_y_int = 0.0f;
    v_z_int = 0.0f;
    p_x_int = 0.0f;
    p_y_int = 0.0f;
    p_z_int = 0.0f;
    a_x_in_W = 0.0f;
    a_y_in_W = 0.0f;
    a_z_in_W = 0.0f;

    // Debug
  }
    // Accelerometer values in the correct frame
  float a_x_1 = cosf(psi)*cosf(theta)*a_x + (sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi))*a_y + (sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi))*a_z;
  float a_y_1 = sinf(psi)*cosf(theta)*a_x + (sinf(phi)*sinf(theta)*sinf(psi) + cosf(psi)*cosf(phi))*a_y + (-1*sinf(phi)*cosf(psi) + sinf(theta)*cosf(phi)*sinf(psi))*a_z;
  float a_z_1 = -1*sinf(theta)*a_x + sinf(phi)*cosf(theta)*a_y + cosf(phi)*cosf(theta)*a_z;

  // Average accelerometer measuremnts before flight
  a_x_0 = ( a_x_0*(sample_count-1) + a_x_1)/sample_count;
  a_y_0 = ( a_y_0*(sample_count-1) + a_y_1)/sample_count;
  a_z_0 = ( a_z_0*(sample_count-1) + a_z_1)/sample_count;
  ++sample_count;
}

void decode_custom_int(uint32_t encoded, uint8_t *int6, float *float_value) {
    // Step 1: Extract the 6-bit integer
    *int6 = (encoded >> 26) & 0x3F;

    // Step 2: Extract and convert the 26-bit custom float
    uint32_t custom_float_bits = encoded & 0x3FFFFFF;
    
    // Extract the sign (1 bit), exponent (6 bits), and mantissa (19 bits)
    int sign = (custom_float_bits >> 25) & 0x1;
    int exponent = (custom_float_bits >> 19) & 0x3F;
    int mantissa = custom_float_bits & 0x7FFFF;

    // Convert to standard IEEE 754 float components
    int standard_exponent = exponent - 31 + 127;  // Adjust bias from 31 to 127
    int standard_mantissa = mantissa << 4;  // Shift mantissa to fill 23 bits

    // Reassemble into a 32-bit IEEE 754 float
    uint32_t f_bits = (sign << 31) | (standard_exponent << 23) | standard_mantissa;
    *float_value = *((float*)&f_bits);  // Interpret bits as float directly
}

void initialize_bounds(){
    for (int i = 0; i < NUM_BOUNDS; i++) {
        bounds[i][0] = default_bounds[i][0];
        bounds[i][1] = default_bounds[i][1];
    }
}

void update_bounds(){
  decode_custom_int(bounds_update, &bounds_update_index, &bounds_update_value);
  if(bounds_update_index != 63){
    bounds[bounds_update_index/2][bounds_update_index%2] = bounds_update_value;
  }
}

void check_motor_safety(){
  // Try to enable bounds setting in flight.py #EXPIRIMENTAL#
  if (safe_to_set_motors){
    // Dynamic Bounds
    bounds[9][0] = default_bounds[9][0] + p_x_des; bounds[9][1] = default_bounds[9][1] +p_x_des;
    bounds[10][0] = default_bounds[10][0] + p_y_des; bounds[10][1] = default_bounds[10][1] + p_y_des;
    bounds[15][0] = default_bounds[15][0] + p_x_des; bounds[15][1] = default_bounds[15][1] + p_x_des;
    bounds[16][0] = default_bounds[16][0] + p_y_des; bounds[16][1] = default_bounds[16][1] + p_y_des;

    // Array of pointers to the variables being checked
    float variables[NUM_BOUNDS] = {
        n_x, n_y, r, psi, theta, phi, 
        w_x, w_y, w_z, p_x, p_y, p_z,
        v_x, v_y, v_z, p_x_int, p_y_int, p_z_int,
        v_x_int, v_y_int, v_z_int,
        a_x_in_W, a_y_in_W, a_z_in_W,
    };

    // Do bounds checking
    for (int i = 0; i < sizeof(variables) / sizeof(variables[0]); i++) {
      if (variables[i] < bounds[i][0] || variables[i] > bounds[i][1]) {
          safe_to_set_motors = 0;
          bounds_violation_index = i;
          bounds_violation_value = variables[i];
      }
    }
  }
}

void backup_state_estimation(){
    // Backup state esitmation
    // Transform into world frame
    a_x_in_W = cosf(psi)*cosf(theta)*a_x + (sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi))*a_y + (sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi))*a_z;
    a_y_in_W = sinf(psi)*cosf(theta)*a_x + (sinf(phi)*sinf(theta)*sinf(psi) + cosf(psi)*cosf(phi))*a_y + (-1*sinf(phi)*cosf(psi) + sinf(theta)*cosf(phi)*sinf(psi))*a_z;
    a_z_in_W = -1*sinf(theta)*a_x + sinf(phi)*cosf(theta)*a_y + cosf(phi)*cosf(theta)*a_z;

    // Euler integration
    // uint16_t max_age = 250;
    float mocap_sum = p_x_mocap + p_y_mocap + p_z_mocap;
    float flow_sum = n_x + n_y;

    // If flow sensor is stale or mocap is stale do x-y estimation
    if (((flow_old == flow_sum) && !use_LED) || ((mocap_old == mocap_sum) && use_mocap)){
      v_x_int += dt*(a_x_in_W-a_x_0);
      p_x_int += dt*v_x_int;
      v_y_int += dt*(a_y_in_W-a_y_0);
      p_y_int += dt*v_y_int;
    } else {
      v_x_int = v_x;
      p_x_int = p_x;
      v_y_int = v_y;
      p_y_int = p_y;
    }

    // If ToF sensors or mocap is stale do z estimation
    if (((r_old == r) && !use_LED) || ((mocap_old == mocap_sum) && use_mocap)){
      v_z_int += dt*(a_z_in_W-a_z_0);
      p_z_int += dt*v_z_int;
    } else {
      v_z_int = v_z;
      p_z_int = p_z;
    }

  if ((flow_old == flow_sum) && !use_LED){++flow_age;} else {flow_age = 0;}
  if ((r_old == r) && !use_LED){++r_age;} else {r_age = 0;}
  if ((mocap_old == mocap_sum) && use_mocap){++mocap_age;} else {mocap_age=0;}

    flow_old = flow_sum;
    r_old = r;
    mocap_old = mocap_sum;
}

// void ae483UpdateWithTOF(void){
//   //
// }

// void ae483UpdateWithFlow(void){
//   //
// }
void ae483UpdateWithTOF(tofMeasurement_t *tof)
{
  if (use_LED) {
    tof_distance = 0.0f;
  } else {
    tof_distance = tof->distance;
  }
  tof_count++;
}

void ae483UpdateWithFlow(flowMeasurement_t *flow)
{
  if (use_LED) {
    flow_dpixelx = 0.0f;
    flow_dpixely = 0.0f;
  } else {
    flow_dpixelx = flow->dpixelx;
    flow_dpixely = flow->dpixely;
  }
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
  psi_des = setpoint->attitude.yaw;

  // Measurements
  w_x = radians(sensors->gyro.x);
  w_y = radians(sensors->gyro.y);
  w_z = radians(sensors->gyro.z);
  a_z = g * sensors->acc.z;
  n_x = flow_dpixelx;
  n_y = flow_dpixely;
  r = tof_distance;

  // New Measurments
  a_x = g * sensors->acc.x;
  a_y = g * sensors->acc.y;

  // Torques by finite difference
  tau_x = J_x * (w_x - w_x_old) / dt;
  tau_y = J_y * (w_y - w_y_old) / dt;
  w_x_old = w_x;
  w_y_old = w_y;

  if (reset) {
    // We don't care when this stuff is called 
    // It must be called at least once before the drone has a p_z_des
    // initialize_observers();
    initialize_bounds();


    // Using Drone World Frame
    // Reset States
    p_x = 0.0f;
    p_y = 0.0f;
    p_z = 0.0f;
    psi = 0.0f;
    theta = 0.0f;
    phi = 0.0f;
    v_x = 0.0f;
    v_y = 0.0f;
    v_z = 0.0f;
    a_z_0 = a_z;  

    // Reset backup state estimation
    v_x_int = 0.0f;
    v_y_int = 0.0f;
    v_z_int = 0.0f;
    p_x_int = 0.0f;
    p_y_int = 0.0f;
    p_z_int = 0.0f;
    a_x_in_W = 0.0f;
    a_y_in_W = 0.0f;
    a_z_in_W = 0.0f;

    // Reset bounds setting and violation tracking
    bounds_violation_index = -1;
    bounds_violation_value = 0.0;
    bounds_update_index = 63;
    bounds_update_value = 0.0;
    bounds_update = 4227858432;

    // Reset backup state estimation tracking stuff
    a_z_0 = a_z;
    flow_age = 0;
    r_age = 0;
    mocap_age = 0;
    flow_old = n_x + n_y;
    mocap_old = p_x_mocap + p_y_mocap + p_z_mocap;
    r_old = r;
    sample_count = 1;

    reset = false;
  }

  // State estimates
  if (use_observer && !use_mocap && !use_LED) {
    // Custom observer without mocap or LED Deck
    STATE_ESTIMATION_WITHOUT_MOCAP_LED;

  } else if (use_observer && use_mocap && !use_LED){
    // Custom observer with mocap but without LED Deck
    STATE_ESTIMATION_WTH_MOCAP_WITHOUT_LED;

  } else if (use_observer && use_mocap && use_LED){
    // Custom observer with mocap and with LED Deck
    STATE_ESTIMATION_WITH_MOCAP_LED;

  } else {
    //Default observer
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

  // Fix estimates before takeoff
  // Create better intial accelerometer offsets
  if (setpoint->mode.z == modeDisable) {

    // We keep initializing to catch any updates to mocap data
    initialize_observers();

    // Update bounds keep doing this up until the moment of flight
    update_bounds();
  } else {
    // Only need to start this state estimation when the drone starts flying
    backup_state_estimation();

    // Only need to check motor safety when the drone starts flying
    if (use_safety){
        check_motor_safety();
    }
  }

// !safe_to_set_motors || 

  if (!safe_to_set_motors || setpoint->mode.z == modeDisable) {
    // If there is no desired position, then all
    // motor power commands should be zero

    m_1 = 0;
    m_2 = 0;
    m_3 = 0;
    m_4 = 0;

  } else {
    // Otherwise, motor power commands should be
    // chosen by the controller
    CONTROLLER;
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
LOG_ADD(LOG_FLOAT,       n_x,                    &n_x) // 
LOG_ADD(LOG_FLOAT,       n_y,                    &n_y)
LOG_ADD(LOG_FLOAT,       r,                      &r)
LOG_ADD(LOG_FLOAT,       a_z,                    &a_z)
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
LOG_ADD(LOG_FLOAT,       p_x_mocap,              &p_x_mocap)
LOG_ADD(LOG_FLOAT,       p_y_mocap,              &p_y_mocap)
LOG_ADD(LOG_FLOAT,       p_z_mocap,              &p_z_mocap)
LOG_ADD(LOG_FLOAT,       psi_mocap,              &psi_mocap)
LOG_ADD(LOG_FLOAT,       theta_mocap,            &theta_mocap)
LOG_ADD(LOG_FLOAT,       phi_mocap,              &phi_mocap)
LOG_GROUP_STOP(ae483log)

LOG_GROUP_START(extravars)
LOG_ADD(LOG_UINT8,       set_motors,             &safe_to_set_motors)
LOG_ADD(LOG_FLOAT,       a_x,                    &a_x)
LOG_ADD(LOG_FLOAT,       a_y,                    &a_y)
LOG_ADD(LOG_FLOAT,       v_x_int,                &v_x_int)
LOG_ADD(LOG_FLOAT,       v_y_int,                &v_y_int)
LOG_ADD(LOG_FLOAT,       v_z_int,                &v_z_int)
LOG_ADD(LOG_FLOAT,       p_x_int,                &p_x_int)
LOG_ADD(LOG_FLOAT,       p_y_int,                &p_y_int)
LOG_ADD(LOG_FLOAT,       p_z_int,                &p_z_int)
LOG_ADD(LOG_FLOAT,       a_x_in_W,               &a_x_in_W)
LOG_ADD(LOG_FLOAT,       a_y_in_W,               &a_y_in_W)
LOG_ADD(LOG_FLOAT,       a_z_in_W,               &a_z_in_W)
LOG_ADD(LOG_UINT32,      flow_age,               &flow_age)
LOG_ADD(LOG_UINT32,      r_age,                  &r_age)
LOG_ADD(LOG_UINT32,      mocap_age,              &mocap_age)
LOG_ADD(LOG_FLOAT,       a_x_0,                  &a_x_0)
LOG_ADD(LOG_FLOAT,       a_y_0,                  &a_y_0)
LOG_ADD(LOG_FLOAT,       a_z_0,                  &a_z_0)
LOG_ADD(LOG_INT8,        violation_index,        &bounds_violation_index)
LOG_ADD(LOG_FLOAT,       violation_value,        &bounds_violation_value)
LOG_ADD(LOG_FLOAT,       psi_des,                &psi_des)
LOG_GROUP_STOP(extravars)

// LOG_GROUP_START(debugvars)
// // ... put debug variables here temporarly ... 
// LOG_GROUP_STOP(debugvars)

//                1234567890123456789012345678 <-- max total length
//                group   .name
PARAM_GROUP_START(ae483par)
PARAM_ADD(PARAM_UINT8,     use_observer,            &use_observer)
PARAM_ADD(PARAM_UINT8,     reset,                   &reset)
PARAM_ADD(PARAM_UINT8,     set_motors,              &safe_to_set_motors)
PARAM_ADD(PARAM_UINT8,     use_safety,              &use_safety)
PARAM_ADD(PARAM_UINT32,    bounds_update,           &bounds_update)
PARAM_ADD(PARAM_UINT8,     use_mocap,               &use_mocap)
PARAM_ADD(PARAM_UINT8,     use_LED,                 &use_LED)
PARAM_GROUP_STOP(ae483par)
