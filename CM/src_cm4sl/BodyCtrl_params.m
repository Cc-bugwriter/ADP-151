
% Initialize parameters automatically when the model gets loaded.
%
% In the File / Model properties dialog of the model, on the tab labeled
% Callbacks, set 'BodyCtrl_params' as the Model's pre-load function.
%
% An alternative way via the Matlab command line would be
%   set_param('BodyCtrl', 'PreLoadFcn', 'BodyCtrl_params');

disp('Executing BodyCtrl_params.m for parameter initialization');

BodyCtrl_k = 300;

