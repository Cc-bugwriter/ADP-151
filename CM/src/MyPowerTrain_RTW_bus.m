%% Motorcycle Bus
if strcmp(vehicle, 'Motorcycle')
    Simulink.Bus.cellToObject({
    % Inputs
    {
	'cm7UserSignal', {
	    {'s0',		1, 'double', -1, 'real', 'Sample'};
	    {'s1',		1, 'double', -1, 'real', 'Sample'};
	    {'s2',		1, 'double', -1, 'real', 'Sample'};
	    {'s3',		1, 'double', -1, 'real', 'Sample'};
	    {'s4',		1, 'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm7PowerTrainWheelIn', {
	    {'Trq_Brake', 		1,'double', -1, 'real', 'Sample'};
	    {'Trq_BrakeReg_trg', 	1,'double', -1, 'real', 'Sample'};
	    {'Trq_T2W', 		1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm7PowerTrainWheelInArray', {
	    {'F', 	1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
	    {'R', 	1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
	}
    }
    {
	'cm7PowerTrainDriveSrc', {
	    {'Trq_trg',	1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm7PowerTrainDriveSrcArray', {
	    {'d0',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
	    {'d1',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
	}
    }
    {
	'cmPowerTrainIn', { ...
	    {'Key', 	1, 'double', -1, 'real', 'Sample'};
	    {'SST', 	1, 'double', -1, 'real', 'Sample'};
	    {'SelectorCtrl',	1, 'double', -1, 'real', 'Sample'};
	    {'GearNoTrg', 	1, 'double', -1, 'real', 'Sample'};
	    {'Clutch', 	1, 'double', -1, 'real', 'Sample'};
	    {'Gas', 	1, 'double', -1, 'real', 'Sample'};
	    {'Brake', 	1, 'double', -1, 'real', 'Sample'};
	    {'Velocity', 	1, 'double', -1, 'real', 'Sample'};
	    {'UserSignal', 	1, 'cm7UserSignal', -1, 'real', 'Sample'};
	    {'WheelIn', 	1, 'cm7PowerTrainWheelInArray', -1, 'real', 'Sample'};
	    {'DriveSrcIn', 	1, 'cm7PowerTrainDriveSrcArray', -1, 'real', 'Sample'};
	}
    }
    % Outputs
    {
	'cm73DTrq', {
	    {'rx',	1,'double', -1, 'real', 'Sample'};
	    {'ry',	1,'double', -1, 'real', 'Sample'};
	    {'rz',	1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm7PowerTrainWheelOut', {
	    {'rot', 		1,'double', -1, 'real', 'Sample'};
	    {'rotv', 		1,'double', -1, 'real', 'Sample'};
	    {'Trq_B2W', 	1,'double', -1, 'real', 'Sample'};
	    {'Trq_Drive', 	1,'double', -1, 'real', 'Sample'};
	    {'Trq_Supp2WC', 	1,'double', -1, 'real', 'Sample'};
	    {'Trq_BrakeReg', 	1,'double', -1, 'real', 'Sample'};
	    {'Trq_BrakeReg_max', 	1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm7PowerTrainWheelOutArray', {
	    {'F', 	1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
	    {'R', 	1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
	}
    }
    {
	'cmPowerTrainOut', { ...
	    {'Ignition', 	1, 'double', -1, 'real', 'Sample'};
	    {'OperationState', 	1, 'double', -1, 'real', 'Sample'};
	    {'OperationError', 	1, 'double', -1, 'real', 'Sample'};
	    {'GearNo', 		1, 'double', -1, 'real', 'Sample'};
	    {'Engine_rotv', 	1, 'double', -1, 'real', 'Sample'};
	    {'Trq_Supp2Bdy1', 	1, 'cm73DTrq', -1, 'real', 'Sample'};
	    {'DL_iDiff_mean', 	1, 'double', -1, 'real', 'Sample'};
	    {'WheelOut', 	1, 'cm7PowerTrainWheelOutArray', -1, 'real', 'Sample'};
	}
    }
    % Cfg Inputs
    {
	'cm7WheelIyyArray', {
	    {'F',	 	1,'double', -1, 'real', 'Sample'};
	    {'R', 		1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cmPowerTrainCfgIn', { ...
	    {'Wheel_Iyy', 	1, 'cm7WheelIyyArray', -1, 'real', 'Sample'};
	    {'WheelRadius',	1, 'double', -1, 'real', 'Sample'};
	    {'Aero_Frcx',	1, 'double', -1, 'real', 'Sample'};
	}
    }
    % Cfg Output
    {
	'cm7FGears', {
	    {'Gear1',		1,'double', -1, 'real', 'Sample'};
	    {'Gear2',		1,'double', -1, 'real', 'Sample'};
	    {'Gear3',		1,'double', -1, 'real', 'Sample'};
	    {'Gear4',		1,'double', -1, 'real', 'Sample'};
	    {'Gear5',		1,'double', -1, 'real', 'Sample'};
	    {'Gear6',		1,'double', -1, 'real', 'Sample'};
	    {'Gear7',		1,'double', -1, 'real', 'Sample'};
	    {'Gear8',		1,'double', -1, 'real', 'Sample'};
	    {'Gear9',		1,'double', -1, 'real', 'Sample'};
	    {'Gear10',		1,'double', -1, 'real', 'Sample'};
	    {'Gear11',		1,'double', -1, 'real', 'Sample'};
	    {'Gear12',		1,'double', -1, 'real', 'Sample'};
	    {'Gear13',		1,'double', -1, 'real', 'Sample'};
	    {'Gear14',		1,'double', -1, 'real', 'Sample'};
	    {'Gear15',		1,'double', -1, 'real', 'Sample'};
	    {'Gear16',		1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm7BGears', {
	    {'Gear1',		1,'double', -1, 'real', 'Sample'};
	    {'Gear2',		1,'double', -1, 'real', 'Sample'};
	    {'Gear3',		1,'double', -1, 'real', 'Sample'};
	    {'Gear4',		1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cmPowerTrainCfgOut', { ...
	    {'StartEngineWithSST',	1, 'double', -1, 'real', 'Sample'};
	    {'PTKind',		1, 'double', -1, 'real', 'Sample'};
	    {'GBKind',		1, 'double', -1, 'real', 'Sample'};
	    {'nFGears',		1,'double', -1, 'real', 'Sample'};
	    {'iFGear',		1,'cm7FGears', -1, 'real', 'Sample'};
	    {'nBGears',		1,'double', -1, 'real', 'Sample'};
	    {'iBGear',		1,'cm7BGears', -1, 'real', 'Sample'};
	    {'DL_iDiff_mean',	1, 'double', -1, 'real', 'Sample'};
	    {'Engine_rotv_max',	1, 'double', -1, 'real', 'Sample'};
	    {'Engine_rotv_idle',	1, 'double', -1, 'real', 'Sample'};
	    {'Velocity_max',	1, 'double', -1, 'real', 'Sample'};
	}
    }
    });
else
%% Car and Truck Bus
    if strcmp(vehicle, 'Car')
	Simulink.Bus.cellToObject({
	{
	    'cm7PowerTrainWheelInArray', {
		{'FL', 	1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
		{'FR', 	1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
		{'RL', 	1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
		{'RR', 	1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
	    }
	}
	{
	    'cm7PowerTrainWheelOutArray', {
		{'FL', 	1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
		{'FR', 	1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
		{'RL', 	1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
		{'RR', 	1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
	    }
	}
	{
	    'cm7WheelIyyArray', {
		{'FL',	 	1,'double', -1, 'real', 'Sample'};
		{'FR', 		1,'double', -1, 'real', 'Sample'};
		{'RL', 		1,'double', -1, 'real', 'Sample'};
		{'RR', 		1,'double', -1, 'real', 'Sample'};
	    }
	}
	{
	    'cm7PowerTrainDriveSrc', {
		{'Trq_trg',	1,'double', -1, 'real', 'Sample'};
	    }
	}
	{
	    'cm7PowerTrainDriveSrcArray', {
		{'d0',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
		{'d1',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
		{'d2',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
		{'d3',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
	    }
	}
	});
    elseif strcmp(vehicle, 'Truck')
	Simulink.Bus.cellToObject({
	{
	    'cm7PowerTrainWheelInArray', {
		{'FL', 	1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
		{'FR', 	1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
		{'RL', 	1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
		{'RR', 	1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
		{'RL2', 1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
		{'RR2', 1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
		{'FL2', 1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
		{'FR2', 1,'cm7PowerTrainWheelIn', -1, 'real', 'Sample'};
	    }
	}
	{
	    'cm7PowerTrainWheelOutArray', {
		{'FL', 	1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
		{'FR', 	1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
		{'RL', 	1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
		{'RR', 	1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
		{'RL2', 1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
		{'RR2', 1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
		{'FL2', 1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
		{'FR2', 1,'cm7PowerTrainWheelOut', -1, 'real', 'Sample'};
	    }
	}
	{
	    'cm7WheelIyyArray', {
		{'FL',	 	1,'double', -1, 'real', 'Sample'};
		{'FR', 		1,'double', -1, 'real', 'Sample'};
		{'RL', 		1,'double', -1, 'real', 'Sample'};
		{'RR', 		1,'double', -1, 'real', 'Sample'};
		{'RL2', 	1,'double', -1, 'real', 'Sample'};
		{'RR2', 	1,'double', -1, 'real', 'Sample'};
		{'FL2', 	1,'double', -1, 'real', 'Sample'};
		{'FR2', 	1,'double', -1, 'real', 'Sample'};
	    }
	}
	{
	    'cm7PowerTrainDriveSrc', {
		{'Trq_trg',	1,'double', -1, 'real', 'Sample'};
	    }
	}
	{
	    'cm7PowerTrainDriveSrcArray', {
		{'d0',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
		{'d1',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
		{'d2',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
		{'d3',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
		{'d4',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
		{'d5',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
		{'d6',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
		{'d7',	1,'cm7PowerTrainDriveSrc', -1, 'real', 'Sample'};
	    }
	}
	});
    end
    Simulink.Bus.cellToObject({
    % Inputs
    {
	'cm7UserSignal', {
	    {'s0',		1, 'double', -1, 'real', 'Sample'};
	    {'s1',		1, 'double', -1, 'real', 'Sample'};
	    {'s2',		1, 'double', -1, 'real', 'Sample'};
	    {'s3',		1, 'double', -1, 'real', 'Sample'};
	    {'s4',		1, 'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm7PowerTrainWheelIn', {
	    {'Trq_Brake', 	1,'double', -1, 'real', 'Sample'};
	    {'Trq_BrakeReg_trg', 	1,'double', -1, 'real', 'Sample'};
	    {'Trq_T2W', 	1,'double', -1, 'real', 'Sample'};
	    {'Trq_WhlBearing', 	1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cmPowerTrainIn', { ...
	    {'Key', 	1, 'double', -1, 'real', 'Sample'};
	    {'SST', 	1, 'double', -1, 'real', 'Sample'};
	    {'SelectorCtrl',	1, 'double', -1, 'real', 'Sample'};
	    {'GearNoTrg', 	1, 'double', -1, 'real', 'Sample'};
	    {'Clutch', 	1, 'double', -1, 'real', 'Sample'};
	    {'Gas', 	1, 'double', -1, 'real', 'Sample'};
	    {'Brake', 	1, 'double', -1, 'real', 'Sample'};
	    {'Velocity', 	1, 'double', -1, 'real', 'Sample'};
	    {'UserSignal', 	1, 'cm7UserSignal', -1, 'real', 'Sample'};
	    {'WheelIn', 	1, 'cm7PowerTrainWheelInArray', -1, 'real', 'Sample'};
	    {'DriveSrcIn', 	1, 'cm7PowerTrainDriveSrcArray', -1, 'real', 'Sample'};
	}
    }
    % Outputs
    {
	'cm72DTrq', {
	    {'rx',	1,'double', -1, 'real', 'Sample'};
	    {'ry',	1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm73DTrq', {
	    {'rx',	1,'double', -1, 'real', 'Sample'};
	    {'ry',	1,'double', -1, 'real', 'Sample'};
	    {'rz',	1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm7PowerTrainWheelOut', {
	    {'rot', 		1,'double', -1, 'real', 'Sample'};
	    {'rotv', 		1,'double', -1, 'real', 'Sample'};
	    {'Trq_B2W', 	1,'double', -1, 'real', 'Sample'};
	    {'Trq_Drive', 	1,'double', -1, 'real', 'Sample'};
	    {'Trq_Supp2WC', 	1,'double', -1, 'real', 'Sample'};
	    {'Trq_BrakeReg', 	1,'double', -1, 'real', 'Sample'};
	    {'Trq_BrakeReg_max', 	1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cmPowerTrainOut', { ...
	    {'Ignition', 	1, 'double', -1, 'real', 'Sample'};
	    {'OperationState', 	1, 'double', -1, 'real', 'Sample'};
	    {'OperationError', 	1, 'double', -1, 'real', 'Sample'};
	    {'GearNo', 		1, 'double', -1, 'real', 'Sample'};
	    {'Engine_rotv', 	1, 'double', -1, 'real', 'Sample'};
	    {'Trq_Supp2Bdy1', 	1, 'cm73DTrq', -1, 'real', 'Sample'};
	    {'Trq_Supp2Bdy1B', 	1, 'cm73DTrq', -1, 'real', 'Sample'};
	    {'Trq_Supp2BdyEng', 	1, 'cm72DTrq', -1, 'real', 'Sample'};
	    {'DL_iDiff_mean', 	1, 'double', -1, 'real', 'Sample'};
	    {'WheelOut', 	1, 'cm7PowerTrainWheelOutArray', -1, 'real', 'Sample'};
	}
    }
    % Cfg Inputs
    {
	'cmPowerTrainCfgIn', { ...
	    {'nWheels',	1, 'double', -1, 'real', 'Sample'};
	    {'Wheel_Iyy', 	1, 'cm7WheelIyyArray', -1, 'real', 'Sample'};
	    {'WheelRadius',	1, 'double', -1, 'real', 'Sample'};
	    {'Aero_Frcx',	1, 'double', -1, 'real', 'Sample'};
	}
    }
    % Cfg Output
    {
	'cm7FGears', {
	    {'Gear1',		1,'double', -1, 'real', 'Sample'};
	    {'Gear2',		1,'double', -1, 'real', 'Sample'};
	    {'Gear3',		1,'double', -1, 'real', 'Sample'};
	    {'Gear4',		1,'double', -1, 'real', 'Sample'};
	    {'Gear5',		1,'double', -1, 'real', 'Sample'};
	    {'Gear6',		1,'double', -1, 'real', 'Sample'};
	    {'Gear7',		1,'double', -1, 'real', 'Sample'};
	    {'Gear8',		1,'double', -1, 'real', 'Sample'};
	    {'Gear9',		1,'double', -1, 'real', 'Sample'};
	    {'Gear10',		1,'double', -1, 'real', 'Sample'};
	    {'Gear11',		1,'double', -1, 'real', 'Sample'};
	    {'Gear12',		1,'double', -1, 'real', 'Sample'};
	    {'Gear13',		1,'double', -1, 'real', 'Sample'};
	    {'Gear14',		1,'double', -1, 'real', 'Sample'};
	    {'Gear15',		1,'double', -1, 'real', 'Sample'};
	    {'Gear16',		1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm7BGears', {
	    {'Gear1',		1,'double', -1, 'real', 'Sample'};
	    {'Gear2',		1,'double', -1, 'real', 'Sample'};
	    {'Gear3',		1,'double', -1, 'real', 'Sample'};
	    {'Gear4',		1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cmPowerTrainCfgOut', { ...
	    {'StartEngineWithSST',	1, 'double', -1, 'real', 'Sample'};
	    {'PTKind',		1, 'double', -1, 'real', 'Sample'};
	    {'GBKind',		1, 'double', -1, 'real', 'Sample'};
	    {'nFGears',		1,'double', -1, 'real', 'Sample'};
	    {'iFGear',		1,'cm7FGears', -1, 'real', 'Sample'};
	    {'nBGears',		1,'double', -1, 'real', 'Sample'};
	    {'iBGear',		1,'cm7BGears', -1, 'real', 'Sample'};
	    {'DL_iDiff_mean',	1, 'double', -1, 'real', 'Sample'};
	    {'Engine_rotv_max',	1, 'double', -1, 'real', 'Sample'};
	    {'Engine_rotv_idle',	1, 'double', -1, 'real', 'Sample'};
	    {'Velocity_max',	1, 'double', -1, 'real', 'Sample'};
	}
    }
    });
end