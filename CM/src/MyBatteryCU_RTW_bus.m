Simulink.Bus.cellToObject({
% Input Bus
{
    'cm10Battery', {
	{'Current', 	1, 'double', -1, 'real', 'Sample'};
	{'AOC', 	1, 'double', -1, 'real', 'Sample'};
	{'Temp', 	1, 'double', -1, 'real', 'Sample'};
	{'Energy', 	1, 'double', -1, 'real', 'Sample'};
	{'Pwr_max', 	1, 'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTBatteryCUIn' , {
	{'Ignition', 		1, 'double', -1, 'real', 'Sample'};
	{'Pwr_LV', 		1, 'double', -1, 'real', 'Sample'};
	{'Pwr_HV1', 		1, 'double', -1, 'real', 'Sample'};
	{'Pwr_HV2', 		1, 'double', -1, 'real', 'Sample'};
	{'Voltage_LV', 		1, 'double', -1, 'real', 'Sample'};
	{'Voltage_HV1', 	1, 'double', -1, 'real', 'Sample'};
	{'Voltage_HV2', 	1, 'double', -1, 'real', 'Sample'};
	{'Pwr_HV1toLV', 	1, 'double', -1, 'real', 'Sample'};
	{'Pwr_HV1toLV_trg', 	1, 'double', -1, 'real', 'Sample'};
	{'Pwr_HV1toHV2', 	1, 'double', -1, 'real', 'Sample'};
	{'Pwr_HV1toLV_max', 	1, 'double', -1, 'real', 'Sample'};
	{'Pwr_HV1toHV2_max', 	1, 'double', -1, 'real', 'Sample'};
	{'Eta_HV1toLV', 	1, 'double', -1, 'real', 'Sample'};
	{'Eta_HV1toHV2', 	1, 'double', -1, 'real', 'Sample'};
	{'BattLV', 		1, 'cm10Battery', -1, 'real', 'Sample'};
	{'BattHV', 		1, 'cm10Battery', -1, 'real', 'Sample'};
    }
}
% Output Bus
{
    'cmPTBatteryCUOut' , {
	{'Status', 	1, 'double', -1, 'real', 'Sample'};
	{'SOC_LV', 	1, 'double', -1, 'real', 'Sample'};
	{'SOC_HV', 	1, 'double', -1, 'real', 'Sample'};
	{'SOH_LV', 	1, 'double', -1, 'real', 'Sample'};
	{'SOH_HV', 	1, 'double', -1, 'real', 'Sample'};
	{'Pwr_HV1toLV_trg', 	1, 'double', -1, 'real', 'Sample'};
	{'BattLV', 	1, 'cm10Battery', -1, 'real', 'Sample'};
	{'BattHV', 	1, 'cm10Battery', -1, 'real', 'Sample'};
    }
}
% CfgInput Bus
{
    'cm10BatteryCfg', {
	{'SOC_min', 	1, 'double', -1, 'real', 'Sample'};
	{'SOC_max', 	1, 'double', -1, 'real', 'Sample'};
	{'Capacity', 	1, 'double', -1, 'real', 'Sample'};
	{'Voltage', 	1, 'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTBatteryCUCfgIn' , {
	{'BattLV', 	1, 'cm10BatteryCfg', -1, 'real', 'Sample'};
	{'BattHV', 	1, 'cm10BatteryCfg', -1, 'real', 'Sample'};
    }
}
});
