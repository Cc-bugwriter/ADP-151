Simulink.Bus.cellToObject({
% Input Bus
{
    'cmPTEngineCUIn', {
	{'Ignition',	1,'double', -1, 'real', 'Sample'};
	{'set_ISC',	1,'double', -1, 'real', 'Sample'};
	{'Load',	1,'double', -1, 'real', 'Sample'};
	{'Trq_trg',	1,'double', -1, 'real', 'Sample'};
	{'Trq',		1,'double', -1, 'real', 'Sample'};
	{'rotv_trg',	1,'double', -1, 'real', 'Sample'};
	{'rotv',	1,'double', -1, 'real', 'Sample'};
	{'rot',		1,'double', -1, 'real', 'Sample'};
	{'FuelFlow',	1,'double', -1, 'real', 'Sample'};
	{'FuelCutOff',	1,'double', -1, 'real', 'Sample'};
    }
}
% Output Bus
{
    'cmPTEngineCUOut', {
	{'Status',	1,'double', -1, 'real', 'Sample'};
	{'FuelCutOff',	1,'double', -1, 'real', 'Sample'};
	{'Engine_on',	1,'double', -1, 'real', 'Sample'};
	{'Load',	1,'double', -1, 'real', 'Sample'};
	{'TrqDrag',	1,'double', -1, 'real', 'Sample'};
	{'TrqFull',	1,'double', -1, 'real', 'Sample'};
	{'TrqOpt',	1,'double', -1, 'real', 'Sample'};
    }
}
% Cfg Input Bus
{
    'cm16PTEngineMap', {
	{'x',	100,'double', -1, 'real', 'Sample'};
	{'z',	100,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTEngineCUCfgIn', {
	{'rotv_off',	1,'double', -1, 'real', 'Sample'};
	{'rotv_max',	1,'double', -1, 'real', 'Sample'};
	{'rotv_idle',	1,'double', -1, 'real', 'Sample'};
	{'rotv_opt',	1,'double', -1, 'real', 'Sample'};
	{'TrqFull',	1,'cm16PTEngineMap', -1, 'real', 'Sample'};
	{'TrqDrag',	1,'cm16PTEngineMap', -1, 'real', 'Sample'};
	{'TrqOpt',	1,'cm16PTEngineMap', -1, 'real', 'Sample'};
    }
}
});
