if strcmp(vehicle, 'Motorcycle')
    Simulink.Bus.cellToObject({
    {
	'cm24Wheels', {
	    {'F',	1,'double', -1, 'real', 'Sample'};
	    {'R',	1,'double', -1, 'real', 'Sample'};
	}
    }
    })
elseif strcmp(vehicle, 'Car')
    Simulink.Bus.cellToObject({
    {
	'cm24Wheels', {
	    {'FL',	1,'double', -1, 'real', 'Sample'};
	    {'FR',	1,'double', -1, 'real', 'Sample'};
	    {'RL',	1,'double', -1, 'real', 'Sample'};
	    {'RR',	1,'double', -1, 'real', 'Sample'};
	}
    }
    })
elseif strcmp(vehicle, 'Truck')
    Simulink.Bus.cellToObject({
    {
	'cm24Wheels', {
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
    })
end
if strcmp(vehicle, 'Motorcycle')
    Simulink.Bus.cellToObject({
    {
	'cmSuspExtFrcsIn', {
	    {'lSpring', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'vSpring', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'lDamp', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'vDamp', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'lBuf', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'vBuf', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	}
    }
    {
	'cmSuspExtFrcsOut', {
	    {'FrcSpring', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'FrcDamp', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'FrcBuf', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	}
    }
    })
elseif strcmp(vehicle, 'Car') || strcmp(vehicle, 'Truck')
    Simulink.Bus.cellToObject({
    {
	'cmSuspExtFrcsIn', {
	    {'lSpring', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'vSpring', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'lDamp', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'vDamp', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'lBuf', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'vBuf', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'lStabi', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'vStabi', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	}
    }
    {
	'cmSuspExtFrcsOut', {
	    {'FrcSpring', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'FrcDamp', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'FrcBuf', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	    {'FrcStabi', 	1, 'cm24Wheels', -1, 'real', 'Sample'};
	}
    }
    })
end
% CfgInput Bus
Simulink.Bus.cellToObject({
{
    'cmSuspExtFrcsCfgIn' , {
	{'VehicleClassId', 	1, 'double', -1, 'real', 'Sample'};
	{'nWheels', 	1, 'double', -1, 'real', 'Sample'};
    }
}
});