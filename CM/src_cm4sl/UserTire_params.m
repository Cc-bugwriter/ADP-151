
% Initialize parameters automatically when the model gets loaded:
%   set_param('UserTire', 'PreLoadFcn', 'UserTire_params');

disp('Executing UserTire_params.m for parameter initialization');


% Make sure this is consistent with Data/Tire/DT_UserTire!

FricCoeff = 1;

% --------------------------------------

Side.FrictionRate =             0.9;
Side.ForceRate =                6.88;
Side.ForceAmplify =             1.0;

Longitudinal.ForceRate =        3.0;
Longitudinal.FrictionRate =     0.9;


