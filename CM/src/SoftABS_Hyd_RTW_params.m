
% Initialize parameters automatically when the model gets loaded:
%   set_param('ABSbraking', 'PreLoadFcn', 'ABSbraking_params');

disp('Executing SoftABS_Hyd_params.m for parameter initialization');



% General
%--------------------------------------
T_sample = 0.001;

ctrl_ABS = 1;         % 1 = on, 2= gepulst mit t_Sensor, 3 = off
triggertime = 0.004; %

switch ctrl_ABS
case 1
t_Sensor = T_sample;
case 2
t_Sensor = triggertime;
case 3
t_Sensor = 1e6;
end

v_min = 5;          % [km/h] lower limit for ABS intervention


% trigger criterion, weighting factors:
%--------------------------------------
f_slip =     1;     %
f_ax   =     0.003; %


% Timefactors Front(V) Axle:
%---------------------------------------
offset=+0.0;
sku_off_VA =               0.03   +offset;
sku_VA =		   0.10   +offset; % 0.13 (0.5) /0.11 (0.7)
sko_VA =		   0.17   +offset;
sko_open_VA =              0.30   +offset;


% LUT valve driving:
%---------------------------------------
Valve_LUTin_VA =[0 sku_off_VA-1e-5 ,sku_off_VA, sku_VA, sku_VA+1e-5, sko_VA-1e-5, sko_VA , sko_open_VA];

%                           off     establish(linear)       hold     reduce(linear)
ValveIn_LUTout_VA      =   [0, 0,   0.2,    0.4,    1.0,    1.0,     1.0,    1.0];
ValveOut_LUTout_VA     =   [0, 0,   0,      0,      0,     0,      0.3,    1.0];



% Timefactors Rear(H) Axle:
%---------------------------------------
offset=+0.0;
sku_off_HA =               0.04   +offset;
sku_HA =		   0.08   +offset;
sko_HA =		   0.15   +offset;
sko_open_HA =              0.30   +offset;


% LUT valve driving:
%---------------------------------------
Valve_LUTin_HA =[0 sku_off_HA-1e-5 ,sku_off_HA, sku_HA, sku_HA+1e-5, sko_HA-1e-5, sko_HA , sko_open_HA];

%                           off     establish(linear)       hold     reduce(linear)
ValveIn_LUTout_HA      =   [0, 0,   0.2,    0.6,    1.0,    1.0,     1.0,    1.0];
ValveOut_LUTout_HA     =   [0, 0,   0,      0,      0,     0,      0.3,    1.0];


