Simulink.Bus.cellToObject({
% Input Bus
{
    'cmPTEngineIn', {
	{'Ignition',	1,'double', -1, 'real', 'Sample'};
	{'FuelCutOff',	1,'double', -1, 'real', 'Sample'};
	{'Load',	1,'double', -1, 'real', 'Sample'};
	{'FuelLevel',	1,'double', -1, 'real', 'Sample'};
	{'rot',		1,'double', -1, 'real', 'Sample'};
	{'rotv',	1,'double', -1, 'real', 'Sample'};
    }
}
% Output Bus
{
    'cmPTEngineOut', {
	{'Trq',		1,'double', -1, 'real', 'Sample'};
	{'Inert',	1,'double', -1, 'real', 'Sample'};
	{'FuelFlow',	1,'double', -1, 'real', 'Sample'};
    }
}
% Cfg Input Bus
{
    'cmPTEngineCfgIn', {
	{'FuelDensity',	1,'double', -1, 'real', 'Sample'};
    }
}
% Cfg Output Bus
{
    'cm15PTEngineMap', {
	{'x',	100,'double', -1, 'real', 'Sample'};
	{'z',	100,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTEngineCfgOut', {
	{'rotv_off',	1,'double', -1, 'real', 'Sample'};
	{'rotv_max',	1,'double', -1, 'real', 'Sample'};
	{'rotv_idle',	1,'double', -1, 'real', 'Sample'};
	{'rotv_opt',	1,'double', -1, 'real', 'Sample'};
	{'TrqFull',	1,'cm15PTEngineMap', -1, 'real', 'Sample'};
	{'TrqDrag',	1,'cm15PTEngineMap', -1, 'real', 'Sample'};
	{'TrqOpt',	1,'cm15PTEngineMap', -1, 'real', 'Sample'};
    }
}
});
