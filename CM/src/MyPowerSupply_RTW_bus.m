Simulink.Bus.cellToObject({
% Input Bus
{
    'cmPTPowerSupplyIn', {
	{'Pwr_HV1toLV_trg',	1,'double', -1, 'real', 'Sample'};
	{'Pwr_LV',		1,'double', -1, 'real', 'Sample'};
	{'Pwr_HV1',		1,'double', -1, 'real', 'Sample'};
	{'Pwr_HV2',		1,'double', -1, 'real', 'Sample'};
    }
}
% Output Bus
{
    'cm21Battery', {
	{'Current',	1,'double', -1, 'real', 'Sample'};
	{'AOC',		1,'double', -1, 'real', 'Sample'};
	{'Energy',	1,'double', -1, 'real', 'Sample'};
	{'Temp',	1,'double', -1, 'real', 'Sample'};
	{'Pwr_max',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTPowerSupplyOut', {
	{'Pwr_HV1toLV',		1,'double', -1, 'real', 'Sample'};
	{'Pwr_HV1toHV2',	1,'double', -1, 'real', 'Sample'};
	{'Pwr_HV1toLV_max', 	1, 'double', -1, 'real', 'Sample'};
	{'Pwr_HV1toHV2_max', 	1, 'double', -1, 'real', 'Sample'};
	{'Eta_HV1toLV', 	1, 'double', -1, 'real', 'Sample'};
	{'Eta_HV1toHV2', 	1, 'double', -1, 'real', 'Sample'};
	{'Voltage_LV',		1,'double', -1, 'real', 'Sample'};
	{'Voltage_HV1',		1,'double', -1, 'real', 'Sample'};
	{'Voltage_HV2',		1,'double', -1, 'real', 'Sample'};
	{'BattLV',		1,'cm21Battery', -1, 'real', 'Sample'};
	{'BattHV',		1,'cm21Battery', -1, 'real', 'Sample'};
    }
}
% Cfg Output Bus
{
    'cm21BatteryCfg', {
	{'SOC_min', 	1, 'double', -1, 'real', 'Sample'};
	{'SOC_max', 	1, 'double', -1, 'real', 'Sample'};
	{'Capacity', 	1, 'double', -1, 'real', 'Sample'};
	{'Voltage', 	1, 'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTPowerSupplyCfgOut' , {
	{'BattLV', 	1, 'cm21BatteryCfg', -1, 'real', 'Sample'};
	{'BattHV', 	1, 'cm21BatteryCfg', -1, 'real', 'Sample'};
    }
}
});
