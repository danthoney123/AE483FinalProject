#ifndef USER_SPECIFIC_LOGIC_H
#define USER_SPECIFIC_LOGIC_H

// User-Specific Constants
#define K_FLOW            4.09255568f      // k flow constant
#define P_Z_EQ            0.5f             // Equlibrium height of z position
#define J_X               0.0000202167f    // principal moment of inertia about x_B axis
#define J_Y               0.000023f        // principal moment of inertia about y_B axis
#define D1D2_INIT         ((2.29967f + 0.28033f) / 100.0f)  // Initialize d1+d2 distances with a constant

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
    tau_x_cmd = 0.00849420f * (p_y - p_y_des) -0.03905284f * phi + 0.00865128f * v_y -0.00269400f * w_x -3.28057629f * tau_x; \
    tau_y_cmd = -0.00849420f * (p_x - p_x_des) -0.04345582f * theta -0.00871641f * v_x -0.00696572f * w_y -5.35385208f * tau_y; \
    tau_z = -0.02395023f * (psi - radians(psi_des)) -0.00229215f * w_z; \
    f_z = -0.70491893f * (p_z - p_z_des) -0.22108596f * v_z + 0.33550200f; \
    m_1 = limitUint16( -4591368.2f * tau_x_cmd -4591368.2f * tau_y_cmd -6811989.1f * tau_z + 151515.2f * f_z ); \
    m_2 = limitUint16( -4591368.2f * tau_x_cmd + 4591368.2f * tau_y_cmd + 6811989.1f * tau_z + 151515.2f * f_z ); \
    m_3 = limitUint16( 4591368.2f * tau_x_cmd + 4591368.2f * tau_y_cmd -6811989.1f * tau_z + 151515.2f * f_z ); \
    m_4 = limitUint16( 4591368.2f * tau_x_cmd -4591368.2f * tau_y_cmd + 6811989.1f * tau_z + 151515.2f * f_z );

#endif // USER_SPECIFIC_LOGIC_H
