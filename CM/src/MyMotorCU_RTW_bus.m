if strcmp(vehicle, 'Motorcycle')
Simulink.Bus.cellToObject({
{
    'cm20PTMotorInArray', {
	{'m0',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
	{'m1',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
    }
}
{
    'cm20PTMotorOut_InArray', {
	{'m0',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
	{'m1',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
    }
}
{
    'cm20PTMotorOutArray', {
	{'m0',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
	{'m1',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
    }
}
{
    'cm20PTMotorArray', {
	{'m0',		1,'cm20PTMotor', -1, 'real', 'Sample'};
	{'m1',		1,'cm20PTMotor', -1, 'real', 'Sample'};
    }
}
});
elseif strcmp(vehicle, 'Car')
Simulink.Bus.cellToObject({
{
    'cm20PTMotorInArray', {
	{'m0',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
	{'m1',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
	{'m2',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
	{'m3',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
    }
}
{
    'cm20PTMotorOut_InArray', {
	{'m0',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
	{'m1',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
	{'m2',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
	{'m3',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
    }
}
{
    'cm20PTMotorOutArray', {
	{'m0',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
	{'m1',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
	{'m2',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
	{'m3',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
    }
}
{
    'cm20PTMotorArray', {
	{'m0',		1,'cm20PTMotor', -1, 'real', 'Sample'};
	{'m1',		1,'cm20PTMotor', -1, 'real', 'Sample'};
	{'m2',		1,'cm20PTMotor', -1, 'real', 'Sample'};
	{'m3',		1,'cm20PTMotor', -1, 'real', 'Sample'};
    }
}
});
elseif strcmp(vehicle, 'Truck')
Simulink.Bus.cellToObject({
{
    'cm20PTMotorInArray', {
	{'m0',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
	{'m1',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
	{'m2',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
	{'m3',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
	{'m4',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
	{'m5',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
	{'m6',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
	{'m7',		1,'cm20PTMotorIn', -1, 'real', 'Sample'};
    }
}
{
    'cm20PTMotorOut_InArray', {
	{'m0',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
	{'m1',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
	{'m2',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
	{'m3',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
	{'m4',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
	{'m5',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
	{'m6',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
	{'m7',		1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
    }
}
{
    'cm20PTMotorOutArray', {
	{'m0',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
	{'m1',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
	{'m2',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
	{'m3',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
	{'m4',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
	{'m5',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
	{'m6',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
	{'m7',		1,'cm20PTMotorOut', -1, 'real', 'Sample'};
    }
}
{
    'cm20PTMotorArray', {
	{'m0',		1,'cm20PTMotor', -1, 'real', 'Sample'};
	{'m1',		1,'cm20PTMotor', -1, 'real', 'Sample'};
	{'m2',		1,'cm20PTMotor', -1, 'real', 'Sample'};
	{'m3',		1,'cm20PTMotor', -1, 'real', 'Sample'};
	{'m4',		1,'cm20PTMotor', -1, 'real', 'Sample'};
	{'m5',		1,'cm20PTMotor', -1, 'real', 'Sample'};
	{'m6',		1,'cm20PTMotor', -1, 'real', 'Sample'};
	{'m7',		1,'cm20PTMotor', -1, 'real', 'Sample'};
    }
}
});
end
Simulink.Bus.cellToObject({
% Input Bus
{
    'cm20PTMotorIn', {
	{'Trq_trg',	1,'double', -1, 'real', 'Sample'};
	{'Trq',		1,'double', -1, 'real', 'Sample'};
	{'rotv_trg',	1,'double', -1, 'real', 'Sample'};
	{'rotv',	1,'double', -1, 'real', 'Sample'};
	{'PwrElec',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm20PTMotorOut_In', {
	{'Load',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTMotorCUIn', {
	{'Ignition',	1,'double', -1, 'real', 'Sample'};
	{'ISGIn',	1,'cm20PTMotorIn', -1, 'real', 'Sample'};
	{'MotorIn',	1,'cm20PTMotorInArray', -1, 'real', 'Sample'};
	{'ISGOut',	1,'cm20PTMotorOut_In', -1, 'real', 'Sample'};
	{'MotorOut',	1,'cm20PTMotorOut_InArray', -1, 'real', 'Sample'};
    }
}
% Output Bus
{
    'cm20PTMotorOut', {
	{'Load',	1,'double', -1, 'real', 'Sample'};
	{'TrqMot_max',	1,'double', -1, 'real', 'Sample'};
	{'TrqGen_max',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTMotorCUOut', {
	{'Status',	1,'double',		-1, 'real', 'Sample'};
	{'ISGOut',	1,'cm20PTMotorOut',	-1, 'real', 'Sample'};
	{'MotorOut',	1,'cm20PTMotorOutArray',	-1, 'real', 'Sample'};
    }
}
% CfgInput Bus
{
    'cm20PTMotorMap', {
	{'x',	100,'double', -1, 'real', 'Sample'};
	{'z',	100,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm20PTMotor', {
	{'Ratio',		1,'double', -1, 'real', 'Sample'};
	{'TrqMot_max',		1,'cm20PTMotorMap', -1, 'real', 'Sample'};
	{'rotv_Mot_max',	1,'double', -1, 'real', 'Sample'};
	{'TrqGen_max',		1,'cm20PTMotorMap', -1, 'real', 'Sample'};
	{'rotv_Gen_max',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTMotorCUCfgIn', {
	{'nMotor',	1,'double',		-1, 'real', 'Sample'};
	{'ISG',		1,'cm20PTMotor',	-1, 'real', 'Sample'};
	{'Motor',	1,'cm20PTMotorArray',	-1, 'real', 'Sample'};
    }
}
});