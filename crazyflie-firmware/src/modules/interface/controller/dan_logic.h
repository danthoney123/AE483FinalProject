#ifndef USER_SPECIFIC_LOGIC_H
#define USER_SPECIFIC_LOGIC_H

// User-Specific Constants
#define K_FLOW            4.09255568f      // k flow constant
#define P_Z_EQ            0.5f             // Equlibrium height of z position
#define J_X               1.40e-05f    // principal moment of inertia about x_B axis
#define J_Y               1.51e-05f        // principal moment of inertia about y_B axis
#define D1D2_INIT         ((2.29967f + 0.28033f) / 100.0f)  // Initialize d1+d2 distances with a constant
#define PI                3.14159265359f
#define HALF_PI           1.57079632679f

// Bounds Stuff
// Bounds Stuff
#define NUM_BOUNDS        27 // Number of bounds variables
static float default_bounds[NUM_BOUNDS][2] = {
    {-80.0f, 80.0f},  // n_x bounds (N_X_LOWER, N_X_UPPER)
    {-80.0f, 80.0f},  // n_y bounds (N_Y_LOWER, N_Y_UPPER)
    {0.0f, 1.0f},     // r bounds (R_LOWER, R_UPPER)
    {-6.3f, 6.3f},    // psi bounds (PSI_LOWER, PSI_UPPER)
    {-0.2f, 0.2f},    // theta bounds (THETA_LOWER, THETA_UPPER)
    {-0.2f, 0.2f},    // phi bounds (PHI_LOWER, PHI_UPPER)
    {-2.0f, 2.0f},    // w_x bounds (W_X_LOWER, W_X_UPPER)
    {-2.0f, 2.0f},    // w_y bounds (W_Y_LOWER, W_Y_UPPER)
    {-4.0f, 4.0f},    // w_z bounds (W_Z_LOWER, W_Z_UPPER)
    {-1.0f, 1.0f},    // p_x dynamic bounds
    {-1.0f, 1.0f},    // p_y dynamic bounds
    {0.0f, 1.5f},     // p_z bounds (P_Z_LOWER, P_Z_UPPER)
    {-1.0f, 1.0f},    // v_x bounds (V_X_LOWER, V_X_UPPER)
    {-1.0f, 1.0f},    // v_y bounds (V_Y_LOWER, V_Y_UPPER)
    {-0.5f, 0.75f},   // v_z bounds (V_Z_LOWER, V_Z_UPPER)
    {-1.5f, 1.5f},    // p_x_int dynamic bounds
    {-1.5f, 1.5f},    // p_y_int dynamic bounds
    {-0.1f, 1.5f},    // p_z_int dynamic bounds (P_Z_INT_LOWER, P_Z_INT_UPPER)
    {-1.5f, 1.5f},    // v_x_int bounds (V_X_INT_LOWER, V_X_INT_UPPER)
    {-1.5f, 1.5f},    // v_y_int bounds (V_Y_INT_LOWER, V_Y_INT_UPPER)
    {-1.0f, 1.0f},    // v_z_int bounds (V_Z_INT_LOWER, V_Z_INT_UPPER)
    {-4.0f, 4.0f},    // a_x_in_W bounds
    {-4.0f, 4.0f},    // a_y_in_W bounds
    {4.0f, 18.0f},    // a_z_in_W bounds
    {-1.0f, 500.0f},  // flow age bounds (number of cycles @ 500 Hz)
    {-1.0f, 500.0f},  // z ranger age bounds (number of cycles @ 500 Hz)
    {-1.0f, 500.0f}   // mocap age bounds (number of cycles @ 500 Hz)
};

// FIXME REPLACE WITH YOUR OWN OBSERVER AND CONTROLLER
// State estimates (this logic will be inserted at each location where STATE_ESTIMATION_X is used)
// Observer without mocap or LED Deck (Lab 7)
#define STATE_ESTIMATION_WITHOUT_MOCAP_LED \
    n_x_err = -1*k_flow*w_y + k_flow*v_x/p_z_eq - n_x; \
    n_y_err = -1*k_flow*w_x + k_flow*v_y/p_z_eq - n_y; \
    r_err = p_z-r; \
    p_x += dt*v_x; \
    p_y += dt*v_y; \
    p_z += dt*(v_z - 21.85558662371406f  * r_err); \
    psi += dt*w_z; \
    theta += dt*(w_y - 0.015089959373186279f *n_x_err); \
    phi += dt*(w_x - -0.013427109974424565f *n_y_err); \
    v_x += dt*(theta*a_z_0 - 0.22131352276793453f *n_x_err); \
    v_y += dt*(-1*phi*a_z_0 - 0.2184729596578784f *n_y_err); \
    v_z += dt*(a_z - a_z_0 - 94.33333333333394f*r_err); 

// Observer with mocap but without LED DECK (Lab 10)
#define STATE_ESTIMATION_WTH_MOCAP_WITHOUT_LED \
    err_0 = -k_flow*w_y + (k_flow*v_x)/p_z_eq - n_x; \
    err_1 = k_flow*w_x + (k_flow*v_y)/p_z_eq - n_y; \
    err_2 = -d_carpet + p_z - r; \
    err_3 = p_x - p_x_mocap; \
    err_4 = p_y - p_y_mocap; \
    err_5 = p_z - p_z_mocap; \
    err_6 = psi - psi_mocap; \
    err_7 = theta - theta_mocap; \
    err_8 = phi - phi_mocap; \
    p_x += dt*(v_x - (0.0030786366954255857f*err_0 + 19.623468144799038f*err_3 + 0.24898129987407316f*err_7)); \
    p_y += dt*(v_y - (0.0062286935450038515f*err_1 + 10.92305497804218f*err_4 + -0.44296342104455017f*err_8)); \
    p_z += dt*(v_z - (21.53590456130722f*err_2 + 0.5369062078996262f*err_5)); \
    psi += dt*(w_z - (1.5000000000000016f*err_6)); \
    theta += dt*(w_y - (0.001561665202129448f*err_0 + 0.6916147218724252f*err_3 + 2.552528071934991f*err_7)); \
    phi += dt*(w_x - (-0.0008101406187475177f*err_1 + -0.13179076989755217f*err_4 + 3.4852583735280604f*err_8)); \
    v_x += dt*(a_z_0*theta - (0.06168328483604743f*err_0 + 31.01715092907147f*err_3 + 5.664138910887472f*err_7)); \
    v_y += dt*(-a_z_0*phi - (0.0715179223697852f*err_1 + 15.383689251205787f*err_4 + -6.7252248898307f*err_8)); \
    v_z += dt*(a_z-a_z_0 - (93.17897306271185f*err_2 + 2.3230214890980805f*err_5)); 

// Observer with mocap and LED DECK (Final Project)
#define STATE_ESTIMATION_WITH_MOCAP_LED \
    err_0 = p_x - p_x_mocap; \
    err_1 = p_y - p_y_mocap; \
    err_2 = p_z - p_z_mocap; \
    err_3 = psi - psi_mocap; \
    err_4 = theta - theta_mocap; \
    err_5 = phi - phi_mocap; \
    p_x += dt*(v_x - (19.88640059336123f*err_0 + 0.26920920812040056f*err_4)); \
    p_y += dt*(v_y - (11.235803213396196f*err_1 + -0.49010250299907365f*err_5)); \
    p_z += dt*(v_z - (6.08230711439294f*err_2)); \
    psi += dt*(w_z - (1.5f*err_3)); \
    theta += dt*(w_y - (0.7478033558900014f*err_0 + 2.560992864248374f*err_4)); \
    phi += dt*(w_x - (-0.1458156207269972f*err_1 + 3.48977583797374f*err_5)); \
    v_x += dt*(a_z_0*theta - (35.8351220544533f*err_0 + 6.043045017090066f*err_4)); \
    v_y += dt*(-a_z_0*phi - (18.46315434939661f*err_1 + -7.217043151186782f*err_5)); \
    v_z += dt*(a_z-a_z_0 - (14.894736842105242f*err_2));

// Controller logic (this code will be inserted at each location where MOTOR_COMMANDS is used)
// Controller and motor logic
#define CONTROLLER \
    tau_x_cmd = 0.01153626f * (p_y - p_y_des) -0.02190339f * phi + 0.00762691f * v_y -0.00314458f * w_x -4.49542642f * tau_x; \
    tau_y_cmd = -0.00815737f * (p_x - p_x_des) -0.02001104f * theta -0.00631934f * v_x -0.00311792f * w_y -4.27664907f * tau_y; \
    tau_z = -0.00125252f * (psi - radians(psi_des)) -0.00026410f * w_z; \
    f_z = -0.35649465f * (p_z - p_z_des) -0.38846142f * v_z + 0.32765400f; \
    m_1 = limitUint16( -3698224.9f * tau_x_cmd -3698224.9f * tau_y_cmd -186567164.2f * tau_z + 147929.0f * f_z ); \
    m_2 = limitUint16( -3698224.9f * tau_x_cmd + 3698224.9f * tau_y_cmd + 186567164.2f * tau_z + 147929.0f * f_z ); \
    m_3 = limitUint16( 3698224.9f * tau_x_cmd + 3698224.9f * tau_y_cmd -186567164.2f * tau_z + 147929.0f * f_z ); \
    m_4 = limitUint16( 3698224.9f * tau_x_cmd -3698224.9f * tau_y_cmd + 186567164.2f * tau_z + 147929.0f * f_z ); 
#endif // USER_SPECIFIC_LOGIC_H