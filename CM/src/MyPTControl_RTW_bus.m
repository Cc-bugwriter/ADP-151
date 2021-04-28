if strcmp(vehicle, 'Motorcycle')
Simulink.Bus.cellToObject({
{
    'cm12Wheels', {
       	{'F',	1,'double', -1, 'real', 'Sample'};
       	{'R',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12MotorInArray', {
       	{'m0',	1,'cm12MotorIn', -1, 'real', 'Sample'};
       	{'m1',	1,'cm12MotorIn', -1, 'real', 'Sample'};
    }
}
{
    'cm12GearBoxInArray', {
       	{'GB_m0',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
       	{'GB_m1',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
    }
}
{
    'cm12PTControlWheelInArray', {
       	{'F',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
       	{'R',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
    }
}
{
    'cm12MotorOutArray', {
       	{'m0',	1,'cm12MotorOut', -1, 'real', 'Sample'};
       	{'m1',	1,'cm12MotorOut', -1, 'real', 'Sample'};
    }
}
{
    'cm12GearBoxOutArray', {
       	{'GB_m0',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
       	{'GB_m1',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
    }
}
{
    'cm12PTControlWheelOutArray', {
       	{'F',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
       	{'R',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
    }
}
{
    'cm12DriveSourcePosArray', {
	{'d0',		1,'double', -1, 'real', 'Sample'};
	{'d1',		1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12MotorArray', {
       	{'m0',		1,'cm12Motor', -1, 'real', 'Sample'};
       	{'m1',		1,'cm12Motor', -1, 'real', 'Sample'};
    }
}
{
    'cm12GearBoxArray', {
       	{'GB_m0',	1,'cm12GearBox', -1, 'real', 'Sample'};
       	{'GB_m1',	1,'cm12GearBox', -1, 'real', 'Sample'};
    }
}
{
    'cm12DriveSrcInArray', {
	{'d0',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
	{'d1',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
    }
}
});
elseif strcmp(vehicle, 'Car')
Simulink.Bus.cellToObject({
% Inputs Bus
{
    'cm12Wheels', {
       	{'FL',	1,'double', -1, 'real', 'Sample'};
       	{'FR',	1,'double', -1, 'real', 'Sample'};
       	{'RL',	1,'double', -1, 'real', 'Sample'};
       	{'RR',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12MotorInArray', {
       	{'m0',	1,'cm12MotorIn', -1, 'real', 'Sample'};
       	{'m1',	1,'cm12MotorIn', -1, 'real', 'Sample'};
       	{'m2',	1,'cm12MotorIn', -1, 'real', 'Sample'};
       	{'m3',	1,'cm12MotorIn', -1, 'real', 'Sample'};
    }
}
{
    'cm12GearBoxInArray', {
       	{'GB_m0',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
       	{'GB_m1',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
       	{'GB_m2',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
       	{'GB_m3',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
    }
}
{
    'cm12PTControlWheelInArray', {
       	{'FL',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
       	{'FR',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
       	{'RL',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
       	{'RR',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
    }
}
{
    'cm12MotorOutArray', {
       	{'m0',	1,'cm12MotorOut', -1, 'real', 'Sample'};
       	{'m1',	1,'cm12MotorOut', -1, 'real', 'Sample'};
       	{'m2',	1,'cm12MotorOut', -1, 'real', 'Sample'};
       	{'m3',	1,'cm12MotorOut', -1, 'real', 'Sample'};
    }
}
{
    'cm12GearBoxOutArray', {
       	{'GB_m0',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
       	{'GB_m1',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
       	{'GB_m2',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
       	{'GB_m3',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
    }
}
{
    'cm12PTControlWheelOutArray', {
       	{'FL',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
       	{'FR',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
       	{'RL',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
       	{'RR',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
    }
}
{
    'cm12DriveSourcePosArray', {
	{'d0',		1,'double', -1, 'real', 'Sample'};
	{'d1',		1,'double', -1, 'real', 'Sample'};
	{'d2',		1,'double', -1, 'real', 'Sample'};
	{'d3',		1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12MotorArray', {
       	{'m0',		1,'cm12Motor', -1, 'real', 'Sample'};
       	{'m1',		1,'cm12Motor', -1, 'real', 'Sample'};
       	{'m2',		1,'cm12Motor', -1, 'real', 'Sample'};
       	{'m3',		1,'cm12Motor', -1, 'real', 'Sample'};
    }
}
{
    'cm12GearBoxArray', {
       	{'GB_m0',	1,'cm12GearBox', -1, 'real', 'Sample'};
       	{'GB_m1',	1,'cm12GearBox', -1, 'real', 'Sample'};
       	{'GB_m2',	1,'cm12GearBox', -1, 'real', 'Sample'};
       	{'GB_m3',	1,'cm12GearBox', -1, 'real', 'Sample'};
    }
}
{
    'cm12DriveSrcInArray', {
	{'d0',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
	{'d1',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
	{'d2',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
	{'d3',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
    }
}
});
elseif strcmp(vehicle, 'Truck')
Simulink.Bus.cellToObject({
{
    'cm12Wheels', {
       	{'FL',	1,'double', -1, 'real', 'Sample'};
       	{'FR',	1,'double', -1, 'real', 'Sample'};
       	{'RL',	1,'double', -1, 'real', 'Sample'};
       	{'RR',	1,'double', -1, 'real', 'Sample'};
	{'RL2',	1,'double', -1, 'real', 'Sample'};
       	{'RR2',	1,'double', -1, 'real', 'Sample'};
       	{'FL2',	1,'double', -1, 'real', 'Sample'};
       	{'FR2',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12MotorInArray', {
       	{'m0',	1,'cm12MotorIn', -1, 'real', 'Sample'};
       	{'m1',	1,'cm12MotorIn', -1, 'real', 'Sample'};
       	{'m2',	1,'cm12MotorIn', -1, 'real', 'Sample'};
       	{'m3',	1,'cm12MotorIn', -1, 'real', 'Sample'};
       	{'m4',	1,'cm12MotorIn', -1, 'real', 'Sample'};
       	{'m5',	1,'cm12MotorIn', -1, 'real', 'Sample'};
       	{'m6',	1,'cm12MotorIn', -1, 'real', 'Sample'};
       	{'m7',	1,'cm12MotorIn', -1, 'real', 'Sample'};
    }
}
{
    'cm12GearBoxInArray', {
       	{'GB_m0',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
       	{'GB_m1',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
       	{'GB_m2',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
       	{'GB_m3',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
       	{'GB_m4',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
       	{'GB_m5',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
       	{'GB_m6',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
       	{'GB_m7',	1,'cm12GearBoxIn', -1, 'real', 'Sample'};
    }
}
{
    'cm12PTControlWheelInArray', {
       	{'FL',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
       	{'FR',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
       	{'RL',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
       	{'RR',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
       	{'RL2',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
       	{'RR2',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
       	{'FL2',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
       	{'FR2',	1,'cm12PTControlWheelIn', -1, 'real', 'Sample'};
    }
}
{
    'cm12MotorOutArray', {
       	{'m0',	1,'cm12MotorOut', -1, 'real', 'Sample'};
       	{'m1',	1,'cm12MotorOut', -1, 'real', 'Sample'};
       	{'m2',	1,'cm12MotorOut', -1, 'real', 'Sample'};
       	{'m3',	1,'cm12MotorOut', -1, 'real', 'Sample'};
       	{'m4',	1,'cm12MotorOut', -1, 'real', 'Sample'};
       	{'m5',	1,'cm12MotorOut', -1, 'real', 'Sample'};
       	{'m6',	1,'cm12MotorOut', -1, 'real', 'Sample'};
       	{'m7',	1,'cm12MotorOut', -1, 'real', 'Sample'};
    }
}
{
    'cm12GearBoxOutArray', {
       	{'GB_m0',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
       	{'GB_m1',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
       	{'GB_m2',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
       	{'GB_m3',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
       	{'GB_m4',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
       	{'GB_m5',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
       	{'GB_m6',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
       	{'GB_m7',	1,'cm12GearBoxOut', -1, 'real', 'Sample'};
    }
}
{
    'cm12PTControlWheelOutArray', {
       	{'FL',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
       	{'FR',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
       	{'RL',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
       	{'RR',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
       	{'RL2',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
       	{'RR2',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
       	{'FL2',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
       	{'FR2',	1,'cm12PTControlWheelOut', -1, 'real', 'Sample'};
    }
}
{
    'cm12DriveSourcePosArray', {
	{'d0',		1,'double', -1, 'real', 'Sample'};
	{'d1',		1,'double', -1, 'real', 'Sample'};
	{'d2',		1,'double', -1, 'real', 'Sample'};
	{'d3',		1,'double', -1, 'real', 'Sample'};
	{'d4',		1,'double', -1, 'real', 'Sample'};
	{'d5',		1,'double', -1, 'real', 'Sample'};
	{'d6',		1,'double', -1, 'real', 'Sample'};
	{'d7',		1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12MotorArray', {
       	{'m0',		1,'cm12Motor', -1, 'real', 'Sample'};
       	{'m1',		1,'cm12Motor', -1, 'real', 'Sample'};
       	{'m2',		1,'cm12Motor', -1, 'real', 'Sample'};
       	{'m3',		1,'cm12Motor', -1, 'real', 'Sample'};
       	{'m4',		1,'cm12Motor', -1, 'real', 'Sample'};
       	{'m5',		1,'cm12Motor', -1, 'real', 'Sample'};
       	{'m6',		1,'cm12Motor', -1, 'real', 'Sample'};
       	{'m7',		1,'cm12Motor', -1, 'real', 'Sample'};
    }
}
{
    'cm12GearBoxArray', {
       	{'GB_m0',	1,'cm12GearBox', -1, 'real', 'Sample'};
       	{'GB_m1',	1,'cm12GearBox', -1, 'real', 'Sample'};
       	{'GB_m2',	1,'cm12GearBox', -1, 'real', 'Sample'};
       	{'GB_m3',	1,'cm12GearBox', -1, 'real', 'Sample'};
       	{'GB_m4',	1,'cm12GearBox', -1, 'real', 'Sample'};
       	{'GB_m5',	1,'cm12GearBox', -1, 'real', 'Sample'};
       	{'GB_m6',	1,'cm12GearBox', -1, 'real', 'Sample'};
       	{'GB_m7',	1,'cm12GearBox', -1, 'real', 'Sample'};
    }
}
{
    'cm12DriveSrcInArray', {
	{'d0',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
	{'d1',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
	{'d2',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
	{'d3',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
	{'d4',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
	{'d5',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
	{'d6',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
	{'d7',	1,'cm12DriveSrcIn', -1, 'real', 'Sample'};
    }
}
});
end
Simulink.Bus.cellToObject({
% Inputs Bus
{
    'cm12UserSignal', {
	{'s0',		1, 'double', -1, 'real', 'Sample'};
	{'s1',		1, 'double', -1, 'real', 'Sample'};
	{'s2',		1, 'double', -1, 'real', 'Sample'};
	{'s3',		1, 'double', -1, 'real', 'Sample'};
	{'s4',		1, 'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12EngineIn', { ...
    	{'Engine_on',	1,'double', -1, 'real', 'Sample'};
    	{'rotv',	1,'double', -1, 'real', 'Sample'};
    	{'Trq',		1,'double', -1, 'real', 'Sample'};
    	{'TrqDrag',	1,'double', -1, 'real', 'Sample'};
    	{'TrqFull',	1,'double', -1, 'real', 'Sample'};
    	{'TrqOpt',	1,'double', -1, 'real', 'Sample'};
    	{'FuelFlow',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12MotorIn', { ...
    	{'rotv',	1,'double', -1, 'real', 'Sample'};
    	{'Trq',		1,'double', -1, 'real', 'Sample'};
    	{'TrqMot_max',	1,'double', -1, 'real', 'Sample'};
    	{'TrqGen_max',	1,'double', -1, 'real', 'Sample'};
    	{'PwrElec',	1,'double', -1, 'real', 'Sample'};
    	{'i_M2W',	1,'cm12Wheels', -1, 'real', 'Sample'};
    }
}
{
    'cm12ClutchIn', {
    	{'Pos',		1,'double', -1, 'real', 'Sample'};
    	{'rotv_in',	1,'double', -1, 'real', 'Sample'};
    	{'rotv_out',	1,'double', -1, 'real', 'Sample'};
    	{'Trq_in',	1,'double', -1, 'real', 'Sample'};
    	{'Trq_out',	1,'double', -1, 'real', 'Sample'};
    	{'i_TrqIn2Out',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12GearBoxIn', {
       	{'GearNo',	1,'double', -1, 'real', 'Sample'};
       	{'GearNo_dis',	1,'double', -1, 'real', 'Sample'};
       	{'Trq_DriveSrc_trg', 1,'double', -1, 'real', 'Sample'};
       	{'i',		1,'double', -1, 'real', 'Sample'};
       	{'rotv_in',	1,'double', -1, 'real', 'Sample'};
       	{'rotv_out',	1,'double', -1, 'real', 'Sample'};
       	{'Trq_in',	1,'double', -1, 'real', 'Sample'};
       	{'Trq_out',	1,'double', -1, 'real', 'Sample'};
       	{'Clutch',	1,'cm12ClutchIn', -1, 'real', 'Sample'};
       	{'Clutch_dis',	1,'cm12ClutchIn', -1, 'real', 'Sample'};
    }
}
{
    'cm12BattIn', {
       	{'SOC',	1,'double', -1, 'real', 'Sample'};
       	{'SOH',	1,'double', -1, 'real', 'Sample'};
       	{'Current',	1,'double', -1, 'real', 'Sample'};
       	{'AOC',	1,'double', -1, 'real', 'Sample'};
       	{'Temp',	1,'double', -1, 'real', 'Sample'};
       	{'Energy',	1,'double', -1, 'real', 'Sample'};
       	{'Pwr_max',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12PwrSupplyIn', {
       	{'Pwr_LV',	1,'double', -1, 'real', 'Sample'};
       	{'Pwr_HV1',	1,'double', -1, 'real', 'Sample'};
       	{'Pwr_HV2',	1,'double', -1, 'real', 'Sample'};
       	{'Voltage_LV',	1,'double', -1, 'real', 'Sample'};
       	{'Voltage_HV1',	1,'double', -1, 'real', 'Sample'};
       	{'Voltage_HV2',	1,'double', -1, 'real', 'Sample'};
       	{'Pwr_HV1toLV',	1,'double', -1, 'real', 'Sample'};
       	{'Pwr_HV1toHV2',1,'double', -1, 'real', 'Sample'};
	{'Pwr_HV1toLV_max', 	1, 'double', -1, 'real', 'Sample'};
	{'Pwr_HV1toHV2_max', 	1, 'double', -1, 'real', 'Sample'};
	{'Eta_HV1toLV', 	1, 'double', -1, 'real', 'Sample'};
	{'Eta_HV1toHV2', 	1, 'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12PTControlWheelIn', {
	{'Trq_BrakeReg_trg', 1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12DriveSrcIn', {
	{'Trq_trg', 1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTControlIn', {
	{'Key', 	1, 'double', -1, 'real', 'Sample'}; ...
	{'SST', 	1, 'double', -1, 'real', 'Sample'}; ...
	{'SelectorCtrl',1, 'double', -1, 'real', 'Sample'}; ...
	{'GearNoTrg', 	1, 'double', -1, 'real', 'Sample'}; ...
	{'Clutch', 	1,'double', -1, 'real', 'Sample'}; ...
	{'Gas', 	1, 'double', -1, 'real', 'Sample'}; ...
	{'Brake', 	1, 'double', -1, 'real', 'Sample'}; ...
	{'Velocity', 	1, 'double', -1, 'real', 'Sample'}; ...
	{'UserSignal', 	1, 'cm12UserSignal', -1, 'real', 'Sample'};
	{'ECU_Status', 	1, 'double', -1, 'real', 'Sample'};
	{'EngineIn', 	1, 'cm12EngineIn', -1, 'real', 'Sample'};
	{'MCU_Status', 	1, 'double', -1, 'real', 'Sample'};
	{'ISGIn', 	1, 'cm12MotorIn', -1, 'real', 'Sample'};
	{'MotorIn', 	1, 'cm12MotorInArray', -1, 'real', 'Sample'};
	{'TCU_Status', 	1, 'double', -1, 'real', 'Sample'};
	{'ClutchIn', 	1, 'cm12ClutchIn', -1, 'real', 'Sample'};
	{'GearBoxIn', 	1, 'cm12GearBoxIn', -1, 'real', 'Sample'};
	{'GearBoxM_In', 1, 'cm12GearBoxInArray', -1, 'real', 'Sample'};
	{'BCU_Status', 	1, 'double', -1, 'real', 'Sample'};
	{'BattLVIn', 	1, 'cm12BattIn', -1, 'real', 'Sample'};
	{'BattHVIn', 	1, 'cm12BattIn', -1, 'real', 'Sample'};
	{'PwrSupplyIn', 1, 'cm12PwrSupplyIn', -1, 'real', 'Sample'};
	{'WheelIn', 	1, 'cm12PTControlWheelInArray', -1, 'real', 'Sample'};
	{'DriveSrcIn', 	1, 'cm12DriveSrcInArray', -1, 'real', 'Sample'};
    }
}
% Output Bus
{
    'cm12EngineOut', {
    	{'set_ISC',	1,'double', -1, 'real', 'Sample'};
    	{'FuelCutOff',	1,'double', -1, 'real', 'Sample'};
    	{'Load',	1,'double', -1, 'real', 'Sample'};
    	{'Trq_trg',	1,'double', -1, 'real', 'Sample'};
    	{'rotv_trg',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12MotorOut', {
    	{'Load',	1,'double', -1, 'real', 'Sample'};
    	{'Trq_trg',	1,'double', -1, 'real', 'Sample'};
    	{'rotv_trg',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12ClutchOut', {
    	{'Pos',	1,'double', -1, 'real', 'Sample'};
    	{'rotv_out_trg',	1,'double', -1, 'real', 'Sample'};
    	{'Trq_out_trg',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12GearBoxOut', {
    	{'GearNoTrg',	1,'double', -1, 'real', 'Sample'};
    	{'GearNoTrg_dis',	1,'double', -1, 'real', 'Sample'};
    	{'i_trg',	1,'double', -1, 'real', 'Sample'};
    	{'set_ParkBrake',1,'double', -1, 'real', 'Sample'};
    	{'rotv_in_trg',	1,'double', -1, 'real', 'Sample'};
    	{'Trq_out_trg',	1,'double', -1, 'real', 'Sample'};
    	{'Clutch',	1,'cm12ClutchOut', -1, 'real', 'Sample'};
    	{'Clutch_dis',	1,'cm12ClutchOut', -1, 'real', 'Sample'};
    }
}
{
    'cm12PwrSupplyOut', {
    	{'Pwr_HV1toLV_trg',  1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12PTControlWheelOut', {
    	{'Trq_BrakeReg',  	1,'double', -1, 'real', 'Sample'};
    	{'Trq_BrakeReg_max', 	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTControlOut', { ...
       	{'OperationState',	1,'double', 		-1, 'real', 'Sample'};
       	{'OperationError',	1,'double', 		-1, 'real', 'Sample'};
       	{'StrategyMode',	1,'double', 		-1, 'real', 'Sample'};
       	{'Ignition',		1,'double', 		-1, 'real', 'Sample'};
       	{'EngineOut',		1,'cm12EngineOut', 	-1, 'real', 'Sample'};
	{'ISGOut',		1,'cm12MotorOut', 	-1, 'real', 'Sample'};
       	{'MotorOut',		1,'cm12MotorOutArray',	-1, 'real', 'Sample'};
       	{'ClutchOut',		1,'cm12ClutchOut', 	-1, 'real', 'Sample'};
       	{'GearBoxOut',		1,'cm12GearBoxOut', 	-1, 'real', 'Sample'};
       	{'GearBoxM_Out',	1,'cm12GearBoxOutArray',-1, 'real', 'Sample'};
       	{'PwrSupplyOut',	1,'cm12PwrSupplyOut', 	-1, 'real', 'Sample'};
       	{'WheelOut',		1,'cm12PTControlWheelOutArray', 	-1, 'real', 'Sample'};
    }
}
% CfgInput Bus
{
    'cm12Batt', {
       	{'SOC_min',	1,'double', -1, 'real', 'Sample'};
       	{'SOC_max',	1,'double', -1, 'real', 'Sample'};
       	{'Capacity',	1,'double', -1, 'real', 'Sample'};
       	{'Voltage',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12PTEngineMap', {
	{'x',	100,'double', -1, 'real', 'Sample'};
	{'z',	100,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12Engine', {
       	{'Fuel_l2kWh',	1,'double', -1, 'real', 'Sample'};
       	{'rotv_max',	1,'double', -1, 'real', 'Sample'};
       	{'rotv_idle',	1,'double', -1, 'real', 'Sample'};
       	{'rotv_opt',	1,'double', -1, 'real', 'Sample'};
	{'TrqFull',	1,'cm12PTEngineMap', -1, 'real', 'Sample'};
	{'TrqDrag',	1,'cm12PTEngineMap', -1, 'real', 'Sample'};
	{'TrqOpt',	1,'cm12PTEngineMap', -1, 'real', 'Sample'};
    }
}
{
    'cm12PTMotorMap', {
	{'x',	100,'double', -1, 'real', 'Sample'};
	{'z',	100,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12Motor', {
	{'Level',		1,'double', -1, 'real', 'Sample'};
	{'Ratio',		1,'double', -1, 'real', 'Sample'};
	{'TrqMot_max',		1,'cm12PTMotorMap', -1, 'real', 'Sample'};
	{'rotv_Mot_max',	1,'double', -1, 'real', 'Sample'};
	{'TrqGen_max',		1,'cm12PTMotorMap', -1, 'real', 'Sample'};
	{'rotv_Gen_max',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12FGearArray', {
	{'Gear1',	1,'double', -1, 'real', 'Sample'};
	{'Gear2',	1,'double', -1, 'real', 'Sample'};
	{'Gear3',	1,'double', -1, 'real', 'Sample'};
	{'Gear4',	1,'double', -1, 'real', 'Sample'};
	{'Gear5',	1,'double', -1, 'real', 'Sample'};
	{'Gear6',	1,'double', -1, 'real', 'Sample'};
	{'Gear7',	1,'double', -1, 'real', 'Sample'};
	{'Gear8',	1,'double', -1, 'real', 'Sample'};
	{'Gear9',	1,'double', -1, 'real', 'Sample'};
	{'Gear10',	1,'double', -1, 'real', 'Sample'};
	{'Gear11',	1,'double', -1, 'real', 'Sample'};
	{'Gear12',	1,'double', -1, 'real', 'Sample'};
	{'Gear13',	1,'double', -1, 'real', 'Sample'};
	{'Gear14',	1,'double', -1, 'real', 'Sample'};
	{'Gear15',	1,'double', -1, 'real', 'Sample'};
	{'Gear16',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12BGearArray', {
	{'Gear1',	1,'double', -1, 'real', 'Sample'};
	{'Gear2',	1,'double', -1, 'real', 'Sample'};
	{'Gear3',	1,'double', -1, 'real', 'Sample'};
	{'Gear4',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm12GearBox', {
    	{'GBKind',	1,'double', -1, 'real', 'Sample'};
    	{'ClKind',	1,'double', -1, 'real', 'Sample'};
    	{'nFGears',	1,'double', -1, 'real', 'Sample'};
    	{'iFGear',	1,'cm12FGearArray', -1, 'real', 'Sample'};
    	{'nBGears',	1,'double', -1, 'real', 'Sample'};
    	{'iBGear',	1,'cm12BGearArray', -1, 'real', 'Sample'};
    }
}
{
    'cm12PlanetGear', {
       	{'Ratio',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTControlCfgIn', { ...
       	{'PTKind',		1,'double', 		-1, 'real', 'Sample'};
       	{'ClKind',		1,'double', 		-1, 'real', 'Sample'};
       	{'nMotor',		1,'double', 		-1, 'real', 'Sample'};
       	{'nGearBoxM',		1,'double', 		-1, 'real', 'Sample'};
       	{'nPlanetGear',		1,'double', 		-1, 'real', 'Sample'};
	{'nWheels',		1,'double', 		-1, 'real', 'Sample'};
	{'WheelRadius',		1,'double', 		-1, 'real', 'Sample'};
	{'Aero_Frcx',		1,'double', 		-1, 'real', 'Sample'};
	{'DL_iDiff_mean',	1,'double', 		-1, 'real', 'Sample'};
       	{'DriveSourcePos',	1,'cm12DriveSourcePosArray',	-1, 'real', 'Sample'};
       	{'BattLV',		1,'cm12Batt', 		-1, 'real', 'Sample'};
       	{'BattHV',		1,'cm12Batt', 		-1, 'real', 'Sample'};
       	{'Engine',		1,'cm12Engine',		-1, 'real', 'Sample'};
       	{'ISG',			1,'cm12Motor',	 	-1, 'real', 'Sample'};
       	{'Motor',		1,'cm12MotorArray', 	-1, 'real', 'Sample'};
       	{'GearBox',		1,'cm12GearBox', 	-1, 'real', 'Sample'};
       	{'GearBoxM',		1,'cm12GearBoxArray', 	-1, 'real', 'Sample'};
       	{'PlanetGear',		1,'cm12PlanetGear',	-1, 'real', 'Sample'};
    }
}
% CfgOutput Bus
{
    'cmPTControlCfgOut', { ...
       	{'StartEngineWithSST',	1,'double', -1, 'real', 'Sample'};
       	{'Velocity_max',	1,'double', -1, 'real', 'Sample'};
    }
}
});
