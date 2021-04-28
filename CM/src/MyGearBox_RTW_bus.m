Simulink.Bus.cellToObject({
% Input Bus
{
    'cm17ClutchIn', {
	{'Pos',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTGearBoxIn', {
	{'GearNoTrg',	1,'double', -1, 'real', 'Sample'};
	{'GearNoTrg_dis', 1,'double', -1, 'real', 'Sample'};
	{'set_ParkBrake', 1,'double', -1, 'real', 'Sample'};
	{'i_trg',	1,'double', -1, 'real', 'Sample'};
	{'Inert_in',	1,'double', -1, 'real', 'Sample'};
	{'rot_out',	1,'double', -1, 'real', 'Sample'};
	{'rotv_out',	1,'double', -1, 'real', 'Sample'};
	{'Trq_in',	1,'double', -1, 'real', 'Sample'};
	{'ClutchIn',	1,'cm17ClutchIn', -1, 'real', 'Sample'};
	{'Clutch_dis_In', 1,'cm17ClutchIn', -1, 'real', 'Sample'};
    }
}
% Output Bus
{
    'cm17ClutchOut', {
	{'rot_in',	1,'double', -1, 'real', 'Sample'};
	{'rotv_in',	1,'double', -1, 'real', 'Sample'};
	{'rot_out',	1,'double', -1, 'real', 'Sample'};
	{'rotv_out',	1,'double', -1, 'real', 'Sample'};
	{'Trq_in',	1,'double', -1, 'real', 'Sample'};
	{'Trq_out',	1,'double', -1, 'real', 'Sample'};
	{'i_TrqIn2Out',	1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTGearBoxOut', {
	{'GearNo',		1,'double', -1, 'real', 'Sample'};
	{'GearNo_dis',		1,'double', -1, 'real', 'Sample'};
	{'i',			1,'double', -1, 'real', 'Sample'};
	{'rot_in',		1,'double', -1, 'real', 'Sample'};
	{'rotv_in',		1,'double', -1, 'real', 'Sample'};
	{'Trq_out',		1,'double', -1, 'real', 'Sample'};
	{'Inert_out',		1,'double', -1, 'real', 'Sample'};
	{'i_TrqIn2Out',		1,'double', -1, 'real', 'Sample'};
	{'Trq_SuppInert',	1,'double', -1, 'real', 'Sample'};
	{'ClutchOut',		1,'cm17ClutchOut', -1, 'real', 'Sample'};
	{'Clutch_dis_Out',	1,'cm17ClutchOut', -1, 'real', 'Sample'};
    }
}
% Cfg Output Bus
{
    'cm17FGears', {
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
    'cm17BGears', {
	{'Gear1',		1,'double', -1, 'real', 'Sample'};
	{'Gear2',		1,'double', -1, 'real', 'Sample'};
	{'Gear3',		1,'double', -1, 'real', 'Sample'};
	{'Gear4',		1,'double', -1, 'real', 'Sample'};
    }
}
{
    'cmPTGearBoxCfgOut', {
	{'GBKind',		1,'double', -1, 'real', 'Sample'};
	{'ClKind',		1,'double', -1, 'real', 'Sample'};
	{'nFGears',		1,'double', -1, 'real', 'Sample'};
	{'iFGear',		1,'cm17FGears', -1, 'real', 'Sample'};
	{'nBGears',		1,'double', -1, 'real', 'Sample'};
	{'iBGear',		1,'cm17BGears', -1, 'real', 'Sample'};
    }
}
});
