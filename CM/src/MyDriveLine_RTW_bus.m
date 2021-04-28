if strcmp(vehicle, 'Car')
Simulink.Bus.cellToObject({
{
    'cm13WheelInArray', {
	{'FL',	1,'cm13WheelIn', -1, 'real', 'Sample'};
	{'FR',	1,'cm13WheelIn', -1, 'real', 'Sample'};
	{'RL',	1,'cm13WheelIn', -1, 'real', 'Sample'};
	{'RR',	1,'cm13WheelIn', -1, 'real', 'Sample'};
    }
}
{
    'cm13DriveInArray', {
	{'d0',	1,'cm13DriveIn', -1, 'real', 'Sample'};
	{'d1',	1,'cm13DriveIn', -1, 'real', 'Sample'};
	{'d2',	1,'cm13DriveIn', -1, 'real', 'Sample'};
	{'d3',	1,'cm13DriveIn', -1, 'real', 'Sample'};
    }
}
{
    'cm13WheelOutArray', {
	{'FL',	1,'cm13WheelOut', -1, 'real', 'Sample'};
	{'FR',	1,'cm13WheelOut', -1, 'real', 'Sample'};
	{'RL',	1,'cm13WheelOut', -1, 'real', 'Sample'};
	{'RR',	1,'cm13WheelOut', -1, 'real', 'Sample'};
    }
}
{
    'cm13Wheels', {
	{'FL',	1,'double', -1, 'real', 'Sample'};
	{'FR',	1,'double', -1, 'real', 'Sample'};
	{'RL',	1,'double', -1, 'real', 'Sample'};
	{'RR',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm13DriveOutArray', {
	{'d0',	1,'cm13DriveOut', -1, 'real', 'Sample'};
	{'d1',	1,'cm13DriveOut', -1, 'real', 'Sample'};
	{'d2',	1,'cm13DriveOut', -1, 'real', 'Sample'};
	{'d3',	1,'cm13DriveOut', -1, 'real', 'Sample'};
    }
}
{
    'cm13DriveSourceArray', {
	{'d0',	1,'double', -1, 'real', 'Sample'};
	{'d1',	1,'double', -1, 'real', 'Sample'};
	{'d2',	1,'double', -1, 'real', 'Sample'};
	{'d3',	1,'double', -1, 'real', 'Sample'};
    }
}
});
elseif strcmp(vehicle, 'Truck')
Simulink.Bus.cellToObject({
{
    'cm13WheelInArray', {
	{'FL',	1,'cm13WheelIn', -1, 'real', 'Sample'};
	{'FR',	1,'cm13WheelIn', -1, 'real', 'Sample'};
	{'RL',	1,'cm13WheelIn', -1, 'real', 'Sample'};
	{'RR',	1,'cm13WheelIn', -1, 'real', 'Sample'};
	{'RL2',	1,'cm13WheelIn', -1, 'real', 'Sample'};
	{'RR2',	1,'cm13WheelIn', -1, 'real', 'Sample'};
	{'FL2',	1,'cm13WheelIn', -1, 'real', 'Sample'};
	{'FR2',	1,'cm13WheelIn', -1, 'real', 'Sample'};
    }
}
{
    'cm13DriveInArray', {
	{'d0',	1,'cm13DriveIn', -1, 'real', 'Sample'};
	{'d1',	1,'cm13DriveIn', -1, 'real', 'Sample'};
	{'d2',	1,'cm13DriveIn', -1, 'real', 'Sample'};
	{'d3',	1,'cm13DriveIn', -1, 'real', 'Sample'};
	{'d4',	1,'cm13DriveIn', -1, 'real', 'Sample'};
	{'d5',	1,'cm13DriveIn', -1, 'real', 'Sample'};
	{'d6',	1,'cm13DriveIn', -1, 'real', 'Sample'};
	{'d7',	1,'cm13DriveIn', -1, 'real', 'Sample'};
    }
}
{
    'cm13WheelOutArray', {
	{'FL',	1,'cm13WheelOut', -1, 'real', 'Sample'};
	{'FR',	1,'cm13WheelOut', -1, 'real', 'Sample'};
	{'RL',	1,'cm13WheelOut', -1, 'real', 'Sample'};
	{'RR',	1,'cm13WheelOut', -1, 'real', 'Sample'};
	{'RL2',	1,'cm13WheelOut', -1, 'real', 'Sample'};
	{'RR2',	1,'cm13WheelOut', -1, 'real', 'Sample'};
	{'FL2',	1,'cm13WheelOut', -1, 'real', 'Sample'};
	{'FR2',	1,'cm13WheelOut', -1, 'real', 'Sample'};
    }
}
{
    'cm13Wheels', {
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
    'cm13DriveOutArray', {
	{'d0',	1,'cm13DriveOut', -1, 'real', 'Sample'};
	{'d1',	1,'cm13DriveOut', -1, 'real', 'Sample'};
	{'d2',	1,'cm13DriveOut', -1, 'real', 'Sample'};
	{'d3',	1,'cm13DriveOut', -1, 'real', 'Sample'};
	{'d4',	1,'cm13DriveOut', -1, 'real', 'Sample'};
	{'d5',	1,'cm13DriveOut', -1, 'real', 'Sample'};
	{'d6',	1,'cm13DriveOut', -1, 'real', 'Sample'};
	{'d7',	1,'cm13DriveOut', -1, 'real', 'Sample'};
    }
}
{
    'cm13DriveSourceArray', {
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
});
end
Simulink.Bus.cellToObject({
% Input Bus
{
    'cm13WheelIn', {
	{'Trq_Brake',		1,'double', -1, 'real', 'Sample'};
	{'Trq_T2W',		1,'double', -1, 'real', 'Sample'};
	{'Trq_WhlBearing',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm13DriveIn', {
	{'Inert_in',	1,'double', -1, 'real', 'Sample'};
	{'Trq_in',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTDriveLineIn', {
	{'WheelIn',	1,'cm13WheelInArray', -1, 'real', 'Sample'};
	{'DriveIn',	1,'cm13DriveInArray', -1, 'real', 'Sample'};
    }
}
% Output Bus
{
    'cm132DTrq', {
	{'rx',	1,'double', -1, 'real', 'Sample'};
	{'ry',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm133DTrq', {
	{'rx',	1,'double', -1, 'real', 'Sample'};
	{'ry',	1,'double', -1, 'real', 'Sample'};
	{'rz',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm13WheelOut', {
	{'rot',		1,'double', -1, 'real', 'Sample'};
	{'rotv',	1,'double', -1, 'real', 'Sample'};
	{'Trq_B2W',	1,'double', -1, 'real', 'Sample'};
	{'Trq_Drive',	1,'double', -1, 'real', 'Sample'};
	{'Trq_Supp2WC',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cm13DriveOut', {
	{'rot_in',	1,'double', -1, 'real', 'Sample'};
	{'rotv_in',	1,'double', -1, 'real', 'Sample'};
	{'i_D2W',	1,'cm13Wheels', -1, 'real', 'Sample'};
    }
}
{
    'cmPTDriveLineOut', {
	{'Trq_Supp2Bdy1',	1,'cm133DTrq', -1, 'real', 'Sample'};
	{'Trq_Supp2Bdy1B',	1,'cm133DTrq', -1, 'real', 'Sample'};
	{'Trq_Supp2BdyEng',	1,'cm132DTrq', -1, 'real', 'Sample'};
	{'iDiff_mean',		1,'double', -1, 'real', 'Sample'};
	{'WheelOut',		1,'cm13WheelOutArray', -1, 'real', 'Sample'};
	{'DriveOut',		1,'cm13DriveOutArray', -1, 'real', 'Sample'};
    }
}
% Cfg Input Bus
{
    'cmPTDriveLineCfgIn', { ...
	{'nWheels',	1, 'double', -1, 'real', 'Sample'};
	{'Wheel_Iyy', 		1, 'cm13Wheels', -1, 'real', 'Sample'};
    }
}
% Cfg Output Bus
{
    'cmPTDriveLineCfgOut', { ...
	{'iDiff_mean',	1, 'double', -1, 'real', 'Sample'};
	{'DriveSourcePos', 	1, 'cm13DriveSourceArray', -1, 'real', 'Sample'};
    }
}
});
