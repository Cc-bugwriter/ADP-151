disp('Executing HydBrakeCU_Parameter.m for parameter initialization');

%--------------------------------------
%--------------------------------------


%== General Parameters ==
%--------------------------------------
g = 9.81;
MPS2KMPH = 3.6;
RADpS2RpM = 60/(2*pi);
RpM2RADpS = (2*pi)/60;
T_sample = 0.001;

v_min = 5;          % [km/h] lower limit for ABS and ESP intervention

% Brake
Area_Front = 23;
Area_Rear = 23;
rBrake = 0.1;
Brake_ratio = 0.7;
% Steering
SteerWheelAng_In = [-15 -8 -7 -6 -5 -4 -3 -2 -1 0 1 2 3 4 5 6 7 8 15];
SteerAng_Out = [-0.6 -0.6 -0.5 -0.41 -0.34 -0.27 -0.2 -0.13 -0.06 0 0.06 0.13 0.2 0.27 0.34 0.41 0.5 0.6 0.6];
%--------------------------------------
%--------------------------------------


%== ABS Parameters ==
%--------------------------------------
ctrl_ABS = 1;         % 1 = on, 2= pulsed with triggertime, 
triggertime = 0.001; %

switch ctrl_ABS
case 1
t_Sensor = T_sample;
case 2
t_Sensor = triggertime;
end

% trigger criterion, weighting factors:
f_slip =     1;     %
f_ax   =     0.003; %

% Timefactors Front(V) Axle:
offset=+0.0;
sku_off_VA =               0.03   +offset;
sku_VA =		   0.10   +offset; % 0.13 (0.5) /0.11 (0.7)
sko_VA =		   0.17   +offset;
sko_open_VA =              0.30   +offset;

% LUT valve driving:
Valve_LUTin_VA =[0 sku_off_VA-1e-5 ,sku_off_VA, sku_VA, sku_VA+1e-5, sko_VA-1e-5, sko_VA , sko_open_VA];

%                           off     establish(linear)       hold     reduce(linear)
ValveIn_LUTout_VA      =   [0, 0,   0.2,    0.4,    1.0,    1.0,     1.0,    1.0];
ValveOut_LUTout_VA     =   [0, 0,   0,      0,      0,     0,      0.3,    1.0];

% Timefactors Rear(H) Axle:
offset=+0.0;
sku_off_HA =               0.04   +offset;
sku_HA =		   0.08   +offset;
sko_HA =		   0.15   +offset;
sko_open_HA =              0.30   +offset;

% LUT valve driving:
Valve_LUTin_HA =[0 sku_off_HA-1e-5 ,sku_off_HA, sku_HA, sku_HA+1e-5, sko_HA-1e-5, sko_HA , sko_open_HA];

%                           off     establish(linear)       hold     reduce(linear)
ValveIn_LUTout_HA      =   [0, 0,   0.2,    0.6,    1.0,    1.0,     1.0,    1.0];
ValveOut_LUTout_HA     =   [0, 0,   0,      0,      0,     0,      0.3,    1.0];

% Velocity and Longslip estimation:
threshold_deceleration = -1; %[m/s²]
%--------------------------------------
%--------------------------------------

 
%== ASR Parameters ==
%--------------------------------------

% ASR: BMR System
ValveIn_aufbau_long = 0; %[-]
ValveOut_aufbau_long = 0; %[-]
ValveIn_aufbau_short = 0.85; %[-]
ValveOut_aufbau_short = 0.85; %[-]
ValveIn_abbau = 1; %[-]
ValveOut_abbau = 1; %[-]

% ASR: differential lock
diff_lock_on = 4; %[m/s]
diff_lock_off = 3; %[m/s]
diff_lock_fac = 1000; %[Nm]

%ASR: MSR Controller
MSR_ctrl_kP = 0.1; %[-]
MSR_ctrl_kI = 0; %[-]
MSR_ctrl_kD = 0; %[-]
MSR_min_throttle_fac = 0.2; %[-]

    % Expert Parameters
    min_throttle = 0.05; %[-]

    delta_vel = 10; %[km/h]
    higherlimit_lowvel = 7; %[km/h]
    lowerlimit_lowvel = 6; %[km/h]
    higherlimit_highvel = 2; %[km/h]
    lowerlimit_highvel = 1; %[km/h]
    highervelocity_fac = 1.04; %[-]
    lowervelocity_fac = 1.02; %[-]

    % ASR: MSR System
    highervel_fac = 0.04; %[-]
    lowervel_fac = 0.02; %[-]
    limit_starting = 10; %[km/h]
    limit_acc = 2; %[m/s²]
    limit_vdiff = 1; %[km/h]

    SH_SL_switch = 15; %[km/h]
    limit_selectlow = 38; %[km/h]

    cat1mode1_higher = 9; %[km/h]
    cat1mode1_lower = 8; %[km/h]
    cat1mode2_higher = 6; %[km/h]
    cat1mode2_lower = 5; %[km/h]
    cat1mode3_higher = 4; %[km/h]
    cat1mode3_lower = 3; %[km/h]
    cat2mode1_higher = 6; %[km/h]
    cat2mode1_lower = 5.5; %[km/h]
    cat2mode2_higher = 3; %[km/h]
    cat2mode2_lower = 2.5; %[km/h]
    cat2mode3_higher = 1; %[km/h]
    cat2mode3_lower = 0.5; %[km/h]
    
    wheelspeed_filter_num = [ones(1,20)/20]; %[-]
    wheelspeed_filter_denom = [1 1]; %[-]

  
%--------------------------------------
%--------------------------------------


%== ESP Parameters ==
%--------------------------------------

PID_kP_yawrate = 6650*0.275; %[-]
PID_kI_yawrate = 0; %[-]
PID_kD_yawrate = 0; %[-]
PID_kP_sideslip = 2550*0.55; %[-]
PID_kI_sideslip = 0; %[-]
PID_kD_sideslip = 0; %[-]

ay_max_on = 4.8;
ay_max_off = 4.0;
ay_per_mu_max_on = 7;
ay_per_mu_max_off = 6;

sidesplip_deviation_max = 3;
yawrate_deviation_max = 1;

    % Expert Parameters
    yawrate_vel_corr = 0.5; %[-]
    sideslip_vel_corr = 5/6; %[-]

    locktime_throttle = 2; %[s]
    ESP_ctrl_kP = 0.4;
    ESP_ctrl_kI = -0.0001;
    ESP_ctrl_kD = 0;
    ESP_ctrl_restore_kI = 0.6;
    ESP_min_throttle_fac = 0;
    brake_valve_ctrl_kP = 0.5;

    vel_char_lim_high = 150; %[km/h]
    vel_char_lim_low = 25; %[km/h]


%--------------------------------------
%--------------------------------------


%== HSA Parameters ==
%--------------------------------------
threshold_eng_trq_on = 30; %[Nm]
threshold_eng_trq_off = 25; %[Nm]
threshold_carpitch_on = 0.1; %[rad]
threshold_carpitch_off = 0.05; %[ad]
threshold_velocity_on = 0.01; %[m/s]
threshold_velocity_off = -0.01; %[m/s]

%--------------------------------------
%--------------------------------------


