if strcmp(vehicle, 'Car')
    Simulink.Bus.cellToObject({
    {
	'cm5Wheels', {
	    {'FL',	1,'double', -1, 'real', 'Sample'};
	    {'FR',	1,'double', -1, 'real', 'Sample'};
	    {'RL',	1,'double', -1, 'real', 'Sample'};
	    {'RR',	1,'double', -1, 'real', 'Sample'};
	}
    }
    {
	'cm5WheelsTrq_stat', {
	    {'FL',	1,'cm5Trq_statMap', -1, 'real', 'Sample'};
	    {'FR',	1,'cm5Trq_statMap', -1, 'real', 'Sample'};
	    {'RL',	1,'cm5Trq_statMap', -1, 'real', 'Sample'};
	    {'RR',	1,'cm5Trq_statMap', -1, 'real', 'Sample'};
	}
    }
    {
	'cm5DriveSrcs', {
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
	'cm5Wheels', {
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
	'cm5WheelsTrq_stat', {
	    {'FL',	1,'cm5Trq_statMap', -1, 'real', 'Sample'};
	    {'FR',	1,'cm5Trq_statMap', -1, 'real', 'Sample'};
	    {'RL',	1,'cm5Trq_statMap', -1, 'real', 'Sample'};
	    {'RR',	1,'cm5Trq_statMap', -1, 'real', 'Sample'};
	    {'RL2',	1,'cm5Trq_statMap', -1, 'real', 'Sample'};
	    {'RR2',	1,'cm5Trq_statMap', -1, 'real', 'Sample'};
	    {'FL2',	1,'cm5Trq_statMap', -1, 'real', 'Sample'};
	    {'FR2',	1,'cm5Trq_statMap', -1, 'real', 'Sample'};
	}
    }
    {
	'cm5DriveSrcs', {
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
    'cmHydBrakeControlIn', {
	{'Pedal',		1, 'double', -1, 'real', 'Sample'};
	{'Park', 		1, 'double', -1, 'real', 'Sample'};
	{'Trq_BrakeReg_max', 	1, 'cm5Wheels', -1, 'real', 'Sample'};
	{'Trq_BrakeReg', 	1, 'cm5Wheels', -1, 'real', 'Sample'};
	{'T_env', 		1, 'double', -1, 'real', 'Sample'};
	{'Trq_WB', 		1, 'cm5Wheels', -1, 'real', 'Sample'};
	{'Trq_PB', 		1, 'cm5Wheels', -1, 'real', 'Sample'};
	{'Rel_SW', 		1, 'double', -1, 'real', 'Sample'};
	{'pMC', 		1, 'double', -1, 'real', 'Sample'};
	{'pWB', 		1, 'cm5Wheels', -1, 'real', 'Sample'};
	{'PuRetVolt', 		1, 'double', -1, 'real', 'Sample'};
	{'PedFrc', 		1, 'double', -1, 'real', 'Sample'};
	{'PedTravel', 		1, 'double', -1, 'real', 'Sample'};
	{'PistTravel', 		1, 'double', -1, 'real', 'Sample'};
	{'DiaphTravel', 	1, 'double', -1, 'real', 'Sample'};
    }
}
% Output Bus
{
    'cm5ValveOutArray', {
       	{'FL_Inlet',	1,'double', -1, 'real', 'Sample'};
       	{'FR_Inlet',	1,'double', -1, 'real', 'Sample'};
       	{'RL_Inlet',	1,'double', -1, 'real', 'Sample'};
       	{'RR_Inlet',	1,'double', -1, 'real', 'Sample'};
       	{'FL_Outlet',	1,'double', -1, 'real', 'Sample'};
       	{'FR_Outlet',	1,'double', -1, 'real', 'Sample'};
	{'RL_Outlet',	1,'double', -1, 'real', 'Sample'};
       	{'RR_Outlet',	1,'double', -1, 'real', 'Sample'};
	{'PV_1',	1,'double', -1, 'real', 'Sample'};
       	{'PV_2',	1,'double', -1, 'real', 'Sample'};
	{'SV_1',	1,'double', -1, 'real', 'Sample'};
       	{'SV_2',	1,'double', -1, 'real', 'Sample'};
	{'V_1',		1,'double', -1, 'real', 'Sample'};
       	{'V_2',		1,'double', -1, 'real', 'Sample'};
	{'V_3',		1,'double', -1, 'real', 'Sample'};
       	{'V_4',		1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmHydBrakeControlOut', {
    	{'Pedal',		1, 'double', -1, 'real', 'Sample'};
	{'Park', 		1, 'double', -1, 'real', 'Sample'};
    	{'Trq_BrakeReg_trg',	1,'cm5Wheels', -1, 'real', 'Sample'};
    	{'Trq_DriveSrc_trg',	1,'cm5DriveSrcs', -1, 'real', 'Sample'};
    	{'Valve',	1,'cm5ValveOutArray', -1, 'real', 'Sample'};
    	{'PumpCtrl',	1,'double', -1, 'real', 'Sample'};
    	{'BooSignal',	1,'double', -1, 'real', 'Sample'};
    }
}
% CfgInput Bus
{
    'cm5Trq_statMap', {
	{'x',	100,'double', -1, 'real', 'Sample'};
	{'z',	100,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmHydBrakeControlCfgIn' , {
	{'VehicleClassId', 	1, 'double', -1, 'real', 'Sample'};
	{'nWheels', 	1, 'double', -1, 'real', 'Sample'};
	{'BrakeKind', 	1, 'double', -1, 'real', 'Sample'};
	{'Trq_stat', 	1, 'cm5WheelsTrq_stat', -1, 'real', 'Sample'};
    }
}
});