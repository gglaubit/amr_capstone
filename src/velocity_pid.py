def vel_pid(vg, vc, dt):
    v_goal = vg # output of neural net in m/s
    v_curr = vc # current velocity in m/s
    prev_err = 0.0
    windup_guard = 10 # needs to be changed??
    kp = 1
    ki = 0.1
    kd = 0.1
    error = v_goal - v_curr
    delta_error = error - prev_err
    p = kp*error
    i = i + error * dt
    if i < -windup_guard:
        i = windup_guard
    elif i > windup_guard:
        i = windup_guard
    d = 0.0 # use acceleration calculated above??
    if delta_time > 0:
        d = delta_error/dt
    prev_err = error
    vel = p + ki*i + kd*d
    return vel
