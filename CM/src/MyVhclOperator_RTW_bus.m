Simulink.Bus.cellToObject({
{
    'cm32VOSteering', { ...
      {'Ang',			1, 'double', -1, 'real', 'Sample'};
      {'AngVel',		1, 'double', -1, 'real', 'Sample'};
      {'AngAcc',		1, 'double', -1, 'real', 'Sample'};
      {'Trq',			1, 'double', -1, 'real', 'Sample'};
    }
}
{
    'cm32VOUserSignal', {
	{'s0',		1, 'double', -1, 'real', 'Sample'};
	{'s1',		1, 'double', -1, 'real', 'Sample'};
	{'s2',		1, 'double', -1, 'real', 'Sample'};
	{'s3',		1, 'double', -1, 'real', 'Sample'};
	{'s4',		1, 'double', -1, 'real', 'Sample'};
    }
}
{
    'cmVhclOperatorIn', {
	{'OperationState_trg', 	1, 'double', -1, 'real', 'Sample'};
	{'OperationState',	1, 'double', -1, 'real', 'Sample'};
	{'OperationError',	1, 'double', -1, 'real', 'Sample'};
	{'Velocity', 		1, 'double', -1, 'real', 'Sample'};
	{'Key',			1, 'double', -1, 'real', 'Sample'};
	{'SST',			1, 'double', -1, 'real', 'Sample'};
	{'Gas',			1, 'double', -1, 'real', 'Sample'};
	{'Brake', 		1, 'double', -1, 'real', 'Sample'};
	{'BrakeLever', 		1, 'double', -1, 'real', 'Sample'};
	{'BrakePark', 		1, 'double', -1, 'real', 'Sample'};
	{'Clutch', 		1, 'double', -1, 'real', 'Sample'};
	{'SelectorCtrl',	1, 'double', -1, 'real', 'Sample'};
	{'GearNo', 		1, 'double', -1, 'real', 'Sample'};
	{'Steering', 		1, 'cm32VOSteering', -1, 'real', 'Sample'};
	{'UserSignal', 		1, 'cm32VOUserSignal', -1, 'real', 'Sample'};
    }
}
{
    'cmVhclOperatorOut', { ...
	{'OperatorFinished',	1, 'double', -1, 'real', 'Sample'};
	{'Key',			1, 'double', -1, 'real', 'Sample'};
	{'SST',			1, 'double', -1, 'real', 'Sample'};
	{'Gas',			1, 'double', -1, 'real', 'Sample'};
	{'Brake', 		1, 'double', -1, 'real', 'Sample'};
	{'BrakeLever', 		1, 'double', -1, 'real', 'Sample'};
	{'BrakePark', 		1, 'double', -1, 'real', 'Sample'};
	{'Clutch', 		1, 'double', -1, 'real', 'Sample'};
	{'SelectorCtrl',	1, 'double', -1, 'real', 'Sample'};
	{'GearNo', 		1, 'double', -1, 'real', 'Sample'};
	{'Steering', 		1, 'cm32VOSteering', -1, 'real', 'Sample'};
	{'UserSignal', 		1, 'cm32VOUserSignal', -1, 'real', 'Sample'};
    }
}
% CfgInput Bus
{
    'cmVhclOperatorCfgIn' , {
	{'GearBoxKind', 	1, 'double', -1, 'real', 'Sample'};
	{'StartEngineWithSST', 	1, 'double', -1, 'real', 'Sample'};
    }
}
});
