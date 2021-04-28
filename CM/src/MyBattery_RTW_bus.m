Simulink.Bus.cellToObject({
{
% Input Bus
    'cmPTBatteryIn' , {
	{'Current', 	1, 'double', -1, 'real', 'Sample'};
    }
}
% Output Bus
{
    'cmPTBatteryOut' , {
	{'Voltage', 	1, 'double', -1, 'real', 'Sample'};
	{'AOC', 	1, 'double', -1, 'real', 'Sample'};
	{'Energy', 	1, 'double', -1, 'real', 'Sample'};
	{'Temp', 	1, 'double', -1, 'real', 'Sample'};
	{'Pwr_max', 	1, 'double', -1, 'real', 'Sample'};
    }
}
% CfgOutput Bus
{
    'cmPTBatteryCfgOut' , {
	{'SOC_init', 	1, 'double', -1, 'real', 'Sample'};
	{'SOC_min', 	1, 'double', -1, 'real', 'Sample'};
	{'SOC_max', 	1, 'double', -1, 'real', 'Sample'};
	{'Capacity', 	1, 'double', -1, 'real', 'Sample'};
	{'Voltage', 	1, 'double', -1, 'real', 'Sample'};
    }
}
});

