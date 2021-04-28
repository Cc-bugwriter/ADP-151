
% Initialize parameters automatically when the model gets loaded:
%   set_param('ABSbraking', 'PreLoadFcn', 'ABSbraking_params');

disp('Executing ABSbraking_params.m for parameter initialization');


f_slip =		1.0;
f_ax =			0.01;

sku =			0.15;
sko =			0.2;

T_F_Pres_Reduce =	0.125;
T_F_Pres_Free =		0.125;
T_F_Pres_Keep =		2.5;

