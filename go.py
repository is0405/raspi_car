import cv2
import numpy as np
import quintic_polynominal_planner as qp
import math

MAX_T = 100.0  # maximum time to the goal [s]
MIN_T = 1.0  # minimum time to the goal[s]

def vision():
    cap = cv2.VideoCapture(0)
    # width
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    # height
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
    # get a frame
    _, frame = cap.read()

    # BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
    lower = np.array([60/2, 58, 80])
    upper = np.array([170/2, 255, 200])
    
    mask = cv2.inRange(hsv, lower, upper)

    # calc 
    mu = cv2.moments(mask, False)

    ok = False
    
    if mu["m00"] != 0:
        x, y = int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
        ok = True
    else :
        x, y = -1, -1

    cv2.circle(frame, (x,y), 4, 255, 2, 4)
    cv2.line(frame, (0,int(H/2)),(W,int(H/2)),(0,0,255),2)
    cv2.line(frame, (int(W/2),0),(int(W/2),H),(0,0,255),2)

    ans_x = 0
    ans_y = 0
    px_count = np.count_nonzero(mask)

    if px_count < 2000:
        ok = False
        
    if ok:
        ans_x = x - W/2
        ans_y = y - H/2

    emergency = False
    if px_count > 70000:
        emergency = True
        
    cv2.imwrite('frame' + str(px_count) + '.png' , frame)
    cv2.imwrite('mask' + str(px_count) + '.png' , mask)

    cap.release()
    cv2.destroyAllWindows()

    return ok, ans_x, ans_y, W, H, emergency

def calclate_y(x, w, start_x):
    
    #distance(m)
    dif_z = 5 - start_x
    #resolution
    res_x = 2592
    res_y = 1944

    #forcal length(m)
    focal_len = 0.0000329#3.29e-5
    #m/px
    px = 0.000000014#1.4e-8

    ans = ( dif_z * res_x * px * x ) / ( focal_len * w )

    return ans

def quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_vel, max_accel, max_jerk, dt):
    
    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)

    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

    for T in np.arange(MIN_T, MAX_T, MIN_T):
        xqp = qp.QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T)
        yqp = qp.QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, T)

        time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

        for t in np.arange(0.0, T + dt, dt):
            time.append(t)
            rx.append(xqp.calc_point(t))
            ry.append(yqp.calc_point(t))

            vx = xqp.calc_first_derivative(t)
            vy = yqp.calc_first_derivative(t)
            v = np.hypot(vx, vy)
            yaw = math.atan2(vy, vx)
            rv.append(v)
            ryaw.append(yaw)

            ax = xqp.calc_second_derivative(t)
            ay = yqp.calc_second_derivative(t)
            a = np.hypot(ax, ay)
            if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
                a *= -1
            ra.append(a)

            jx = xqp.calc_third_derivative(t)
            jy = yqp.calc_third_derivative(t)
            j = np.hypot(jx, jy)
            if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
                j *= -1
            rj.append(j)

        if max([abs(i) for i in rv]) <= max_vel and max([abs(i) for i in ra]) <= max_accel and max([abs(i) for i in rj]) <= max_jerk:
            print("find path!!")
            break

    return time, rx, ry, ryaw, rv, ra, rj

def quinic( start_x ):
    print(__file__ + " start!!")

    ok, ca_x, ca_y, W, H, emergency = vision()
    
    if ok and not emergency:
        ans = calclate_y(ca_x, W, start_x)
        print(ans)
        sx = start_x  # start x position [m]
        sy_l = -0.0625  # start y position [m]
        sy_r = 0.0625
        syaw = np.deg2rad(0.0)  # start yaw angle [rad]
        sv_l = 0.0  # start speed [m/s]
        sa_l = 0.0  # start accel [m/ss]
        sv_r = 0.0  # start speed [m/s]
        sa_r = 0.0  # start accel [m/ss]
    
        gx = 5.0  # goal x position [m]
        gy = ans  # goal y position [m]
        gyaw = np.deg2rad(0.0)  # goal yaw angle [rad]
        gv = 0.0  # goal speed [m/s]
        ga = 0.0  # goal accel [m/ss]

        max_vel = 0.65  # max speed [m/s]
        max_accel = 1.0  # max accel [m/ss]
        max_jerk = 0.7  # max jerk [m/sss]
        dt = 0.1  # time tick [s]

        time_l, x_l, y_l, yaw_l, v_l, a_l, j_l = quintic_polynomials_planner(
            sx, sy_l, syaw, sv_l, sa_l, gx, gy, gyaw, gv, ga, max_vel, max_accel, max_jerk, dt)
        time_r, x_r, y_r, yaw_r, v_r, a_r, j_r = quintic_polynomials_planner(
            sx, sy_r, syaw, sv_r, sa_r, gx, gy, gyaw, gv, ga, max_vel, max_accel, max_jerk, dt)

        return True, v_l, v_r, False
    else:
        return False, 0, 0, emergency

    
