%% Motorcycle Bus
if strcmp(vehicle, 'Motorcycle')
    Simulink.Bus.cellToObject({
    % Input Bus
    {
	'cmBrakeIn', { ...
	    {'Use_pMCPedalInput', 1, 'double', -1, 'real', 'Sample'};
	    {'Use_pMCLeverInput', 1, 'double', -1, 'real', 'Sample'};
	    {'Pedal', 1, 'double', -1, 'real', 'Sample'};
	    {'Lever', 1, 'double', -1, 'real', 'Sample'};
	    {'Park', 1, 'double', -1, 'real', 'Sample'};
	    {'pMCPedal_in', 1, 'double', -1, 'real', 'Sample'};
	    {'pMCLever_in', 1, 'double', -1, 'real', 'Sample'};
	    {'PumpCtrl', 1, 'double', -1, 'real', 'Sample'};
	    {'T_env', 1, 'double', -1, 'real', 'Sample'};
	}
    }
    % Output Bus
    {
	'cm4BrakeCylinderArray', {
	    {'FL_l0', 1, 'double', -1, 'real', 'Sample'};
	    {'FL_l1', 1, 'double', -1, 'real', 'Sample'};
	    {'FL_p0', 1, 'double', -1, 'real', 'Sample'};
	    {'FR_l0', 1, 'double', -1, 'real', 'Sample'};
	    {'FR_l1', 1, 'double', -1, 'real', 'Sample'};
	    {'FR_p0', 1, 'double', -1, 'real', 'Sample'};
	    {'R_l0', 1, 'double', -1, 'real', 'Sample'};
	    {'R_l1', 1, 'double', -1, 'real', 'Sample'};
	    {'R_p0', 1, 'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm4BrakeWheelOut', {
	    {'Trq_WB', 1, 'double', -1, 'real', 'Sample'};
	    {'Trq_PB', 1, 'double', -1, 'real', 'Sample'};
	    {'Trq_ext', 1, 'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm4BrakeWheelOutArray', {
	    {'F',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
	    {'R',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
	}
    }
    {
	'cm4DriveSrcArray', {
	    {'d0',	1,'double', -1, 'real', 'Sample'};
	    {'d1',	1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cmBrakeOut', {
	    {'pMCPedal', 1, 'double', -1, 'real', 'Sample'};
	    {'pMCLever', 1, 'double', -1, 'real', 'Sample'};
	    {'pWB',1,'cm4BrakeCylinderArray', -1, 'real', 'Sample'};
	    {'Trq',1,'cm4BrakeCylinderArray', -1, 'real', 'Sample'};
	    {'WheelOut',1,'cm4BrakeWheelOutArray', -1, 'real', 'Sample'};
	    {'Trq_DriveSrc_trg',1,'cm4DriveSrcArray', -1, 'real', 'Sample'};
	}
    }
    % CfgInput Bus
    {
	'cmBrakeCfgIn' , {
	    {'VehicleClassId', 	1, 'double', -1, 'real', 'Sample'};
	    {'nWheels', 	1, 'double', -1, 'real', 'Sample'};
	}
    }
    })
%% Car and Truck Bus
else
    if strcmp(vehicle, 'Car')
	Simulink.Bus.cellToObject({
	{
	    'cm4BrakeWheelInArray', {
		{'FL',	1,'cm4BrakeWheelIn', -1, 'real', 'Sample'};
		{'FR',	1,'cm4BrakeWheelIn', -1, 'real', 'Sample'};
		{'RL',	1,'cm4BrakeWheelIn', -1, 'real', 'Sample'};
		{'RR',	1,'cm4BrakeWheelIn', -1, 'real', 'Sample'};
	    }
	}
	{
	    'cm4BrakeWheelOutArray', {
		{'FL',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
		{'FR',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
		{'RL',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
		{'RR',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
	    }
	}
	{
	    'cm4DriveSrcArray', {
		{'d0',	1,'double', -1, 'real', 'Sample'};
		{'d1',	1,'double', -1, 'real', 'Sample'};
		{'d2',	1,'double', -1, 'real', 'Sample'};
		{'d3',	1,'double', -1, 'real', 'Sample'};
	    }
	}
	})
    elseif strcmp(vehicle, 'Truck')
	Simulink.Bus.cellToObject({
	{
	    'cm4BrakeWheelInArray', {
		{'FL',	1,'cm4BrakeWheelIn', -1, 'real', 'Sample'};
		{'FR',	1,'cm4BrakeWheelIn', -1, 'real', 'Sample'};
		{'RL',	1,'cm4BrakeWheelIn', -1, 'real', 'Sample'};
		{'RR',	1,'cm4BrakeWheelIn', -1, 'real', 'Sample'};
		{'RL2',	1,'cm4BrakeWheelIn', -1, 'real', 'Sample'};
		{'RR2',	1,'cm4BrakeWheelIn', -1, 'real', 'Sample'};
		{'FL2',	1,'cm4BrakeWheelIn', -1, 'real', 'Sample'};
		{'FR2',	1,'cm4BrakeWheelIn', -1, 'real', 'Sample'};
	    }
	}
	{
	    'cm4BrakeWheelOutArray', {
		{'FL',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
		{'FR',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
		{'RL',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
		{'RR',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
		{'RL2',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
		{'RR2',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
		{'FL2',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
		{'FR2',	1,'cm4BrakeWheelOut', -1, 'real', 'Sample'};
	    }
	}
	{
	    'cm4DriveSrcArray', {
		{'d0',	1,'double', -1, 'real', 'Sample'};
		{'d1',	1,'double', -1, 'real', 'Sample'};
		{'d2',	1,'double', -1, 'real', 'Sample'};
		{'d3',	1,'double', -1, 'real', 'Sample'};
		{'d4',	1,'double', -1, 'real', 'Sample'};
		{'d5',	1,'double', -1, 'real', 'Sample'};
		{'d6',	1,'double', -1, 'real', 'Sample'};
		{'d7',	1,'double', -1, 'real', 'Sample'};
	    }
	}
	})
    end
    Simulink.Bus.cellToObject({
    {
    % Input Bus
	'cm4BrakeWheelIn', {
	    {'Trq_BrakeReg_max',1, 'double', -1, 'real', 'Sample'};
	    {'Trq_BrakeReg',1, 'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cmBrakeIn', { ...
	    {'Pedal',1, 'double', -1, 'real', 'Sample'};
	    {'Park', 1, 'double', -1, 'real', 'Sample'};
	    {'T_env', 1, 'double', -1, 'real', 'Sample'};
	    {'WheelIn',1,'cm4BrakeWheelInArray', -1, 'real', 'Sample'};
	}
    }
    % Output Bus
    {
	'cm4BrakeWheelOut', {
	    {'Trq_WB', 1, 'double', -1, 'real', 'Sample'};
	    {'Trq_PB', 1, 'double', -1, 'real', 'Sample'};
	    {'Trq_BrakeReg_trg', 1, 'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cmBrakeOut', {
	    {'WheelOut',1,'cm4BrakeWheelOutArray', -1, 'real', 'Sample'};
	    {'Trq_DriveSrc_trg',1,'cm4DriveSrcArray', -1, 'real', 'Sample'};
	}
    }
    % CfgInput Bus
    {
	'cmBrakeCfgIn' , {
	    {'VehicleClassId', 	1, 'double', -1, 'real', 'Sample'};
	    {'nWheels', 	1, 'double', -1, 'real', 'Sample'};
	}
    }
    })
end
