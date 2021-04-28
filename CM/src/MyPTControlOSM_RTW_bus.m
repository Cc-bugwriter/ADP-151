Simulink.Bus.cellToObject({
% Inputs Bus
{
    'cm33UserSignal', {
	{'s0',		1, 'double', -1, 'real', 'Sample'};
	{'s1',		1, 'double', -1, 'real', 'Sample'};
	{'s2',		1, 'double', -1, 'real', 'Sample'};
	{'s3',		1, 'double', -1, 'real', 'Sample'};
	{'s4',		1, 'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTControlOSMIn', {
	{'Key', 	1, 'double', -1, 'real', 'Sample'};
	{'SST', 	1, 'double', -1, 'real', 'Sample'};
	{'SelectorCtrl',1, 'double', -1, 'real', 'Sample'};
	{'GearNoTrg', 	1, 'double', -1, 'real', 'Sample'};
	{'Clutch', 	1, 'double', -1, 'real', 'Sample'};
	{'Gas', 	1, 'double', -1, 'real', 'Sample'};
	{'Brake', 	1, 'double', -1, 'real', 'Sample'};
	{'Velocity', 	1, 'double', -1, 'real', 'Sample'};
	{'UserSignal', 	1, 'cm33UserSignal', -1, 'real', 'Sample'};
	{'FuncReturn', 	1, 'double', -1, 'real', 'Sample'};
    }
}
% Output Bus
{
    'cmPTControlOSMOut', { ...
       	{'OperationState',	1,'double', 	-1, 'real', 'Sample'};
       	{'OperationError',	1,'double', 	-1, 'real', 'Sample'};
       	{'OSMFunc',		1,'double', 	-1, 'real', 'Sample'};
    }
}
% CfgInput Bus
{
    'cm33OSMFunc', {
	{'OSMFunc_None',	1,'double', -1, 'real', 'Sample'};
	{'OSMFunc_Absent',	1,'double', -1, 'real', 'Sample'};
	{'OSMFunc_PowerOff',	1,'double', -1, 'real', 'Sample'};
	{'OSMFunc_PowerAcc',	1,'double', -1, 'real', 'Sample'};
	{'OSMFunc_PowerOn',	1,'double', -1, 'real', 'Sample'};
	{'OSMFunc_Start',	1,'double', -1, 'real', 'Sample'};
	{'OSMFunc_SkipStart',	1,'double', -1, 'real', 'Sample'};
	{'OSMFunc_Drive',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTControlOSMCfgIn', { ...
       	{'StartEngineWithSST'	1,'double', 		-1, 'real', 'Sample'};
       	{'GBKind',		1,'double', 		-1, 'real', 'Sample'};
       	{'Func_used',		1,'cm33OSMFunc', 	-1, 'real', 'Sample'};
    }
}
});

