// Follow line function in velocity.c
static void follow_line(context* ctx)
{
    double frame_width = 78;
    double frame_height = 51;
    double window_center = frame_width / 2;
    double linear_velocity=0.7;
    double angular_velocity=-0.6;
    double single_line_steer_scale=0.6;

    double x = 0.0;
    double y = 0.0;

    double steer = 0.0;
    double speed = 0.0;
     
    int num_vectors = 0;
    if(!(ctx->pixy_vector.m0_x0 == 0 && ctx->pixy_vector.m0_x1 == 0 && ctx->pixy_vector.m0_y0 == 0 && ctx->pixy_vector.m0_y1 == 0)) {
        num_vectors++;
    }
    if(!(ctx->pixy_vector.m1_x0 == 0 && ctx->pixy_vector.m1_x1 == 0 && ctx->pixy_vector.m1_y0 == 0 && ctx->pixy_vector.m1_y1 == 0)) {
        num_vectors++;
    }
    
    switch(num_vectors) {
        case 0:
            speed = 0.0;
            steer = 0.0;
            break;
        case 1:
            if(ctx->pixy_vector.m0_x1 > ctx->pixy_vector.m0_x0) {
                x = (ctx->pixy_vector.m0_x1 - ctx->pixy_vector.m0_x0) / frame_width;
                y = (ctx->pixy_vector.m0_y1 - ctx->pixy_vector.m0_y0) / frame_height;
            } else {
                x = (ctx->pixy_vector.m0_x0 - ctx->pixy_vector.m0_x1) / frame_width;
                y = (ctx->pixy_vector.m0_y0 - ctx->pixy_vector.m0_y1) / frame_height;
            }

            if((ctx->pixy_vector.m0_x0 != ctx->pixy_vector.m0_x1) && ((y > 0.0) || (y <0.0))) {
                steer = angular_velocity * (x / y) * single_line_steer_scale;
            } else {
                steer = 0.0;
            }
            speed = linear_velocity;
            break;
        case 2:
            if((ctx->pixy_vector.m1_x0 >= ctx->pixy_vector.m1_x1) && (ctx->pixy_vector.m0_x0 <= ctx->pixy_vector.m0_x1)){
                steer = angular_velocity * (((ctx->pixy_vector.m0_x1 + ctx->pixy_vector.m1_x1) / 2.0) - window_center) / frame_width;
            }else if ((ctx->pixy_vector.m1_x0 < ctx->pixy_vector.m1_x1) && (ctx->pixy_vector.m0_x0 <= ctx->pixy_vector.m0_x1)){
                steer = angular_velocity * (((ctx->pixy_vector.m0_x1 + ctx->pixy_vector.m1_x0) / 2.0) - window_center) / frame_width;
            }else if ((ctx->pixy_vector.m1_x0 > ctx->pixy_vector.m1_x1) && (ctx->pixy_vector.m0_x0 > ctx->pixy_vector.m0_x1)){
                steer = angular_velocity * (((ctx->pixy_vector.m0_x0 + ctx->pixy_vector.m1_x1) / 2.0) - window_center) / frame_width;
            }else {
                steer = angular_velocity * (((ctx->pixy_vector.m0_x0 + ctx->pixy_vector.m1_x0) / 2.0) - window_center) / frame_width;
            }

            speed = linear_velocity;      
            break;
    }
    
    double vel_linear_x = speed * (1 - fabs(2 * steer));  
     
    double turn_angle = 0;
    double omega_fwd = 0;
    double V = vel_linear_x;
    double omega = steer;
    double delta = 0;
    CASADI_FUNC_ARGS(ackermann_steering);
    args[0] = &ctx->wheel_base;
    args[1] = &omega;
    args[2] = &V;
    res[0] = &delta;
    CASADI_FUNC_CALL(ackermann_steering);

    omega_fwd = V / ctx->wheel_radius;
    if (fabs(V) > 0.01) {
        turn_angle = delta;
    }
    b3rb_set_actuators(&ctx->actuators, turn_angle, omega_fwd);
}
