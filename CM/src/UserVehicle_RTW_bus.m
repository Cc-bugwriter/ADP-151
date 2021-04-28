if strcmp(vehicle, 'Motorcycle')
elseif strcmp(vehicle, 'Car')
Simulink.Bus.cellToObject({
  {
    'cm30WheelInArray', {
      {'FL',	 	1,'cm30VehicleWheelIn', -1, 'real', 'Sample'};
      {'FR', 		1,'cm30VehicleWheelIn', -1, 'real', 'Sample'};
      {'RL', 		1,'cm30VehicleWheelIn', -1, 'real', 'Sample'};
      {'RR', 		1,'cm30VehicleWheelIn', -1, 'real', 'Sample'};
    }
  }
  {
    'cm30WheelOutArray', {
      {'FL',	 	1,'cm30VehicleWheelOut', -1, 'real', 'Sample'};
      {'FR', 		1,'cm30VehicleWheelOut', -1, 'real', 'Sample'};
      {'RL', 		1,'cm30VehicleWheelOut', -1, 'real', 'Sample'};
      {'RR', 		1,'cm30VehicleWheelOut', -1, 'real', 'Sample'};
    }
  }
  {
    'cm30BrakeWheelArray', {
      {'FL',	 	1,'cm30VehicleBrakeWheel', -1, 'real', 'Sample'};
      {'FR', 		1,'cm30VehicleBrakeWheel', -1, 'real', 'Sample'};
      {'RL', 		1,'cm30VehicleBrakeWheel', -1, 'real', 'Sample'};
      {'RR', 		1,'cm30VehicleBrakeWheel', -1, 'real', 'Sample'};
    }
  }
  {
    'cm30PTWheelArray', {
      {'FL',	 	1,'cm30VehiclePTWheel', -1, 'real', 'Sample'};
      {'FR', 		1,'cm30VehiclePTWheel', -1, 'real', 'Sample'};
      {'RL', 		1,'cm30VehiclePTWheel', -1, 'real', 'Sample'};
      {'RR', 		1,'cm30VehiclePTWheel', -1, 'real', 'Sample'};
    }
  }
  {
    'cm30WheelArray', {
      {'FL',	 	1,'double', -1, 'real', 'Sample'};
      {'FR', 		1,'double', -1, 'real', 'Sample'};
      {'RL', 		1,'double', -1, 'real', 'Sample'};
      {'RR', 		1,'double', -1, 'real', 'Sample'};
    }
  }
})
elseif strcmp(vehicle, 'Truck')
Simulink.Bus.cellToObject({
  {
    'cm30WheelInArray', {
      {'FL',	 	1,'cm30VehicleWheelIn', -1, 'real', 'Sample'};
      {'FR', 		1,'cm30VehicleWheelIn', -1, 'real', 'Sample'};
      {'RL', 		1,'cm30VehicleWheelIn', -1, 'real', 'Sample'};
      {'RR', 		1,'cm30VehicleWheelIn', -1, 'real', 'Sample'};
      {'RL2', 		1,'cm30VehicleWheelIn', -1, 'real', 'Sample'};
      {'RR2', 		1,'cm30VehicleWheelIn', -1, 'real', 'Sample'};
      {'FL2',	 	1,'cm30VehicleWheelIn', -1, 'real', 'Sample'};
      {'FR2', 		1,'cm30VehicleWheelIn', -1, 'real', 'Sample'};
    }
  }
  {
    'cm30WheelOutArray', {
      {'FL',	 	1,'cm30VehicleWheelOut', -1, 'real', 'Sample'};
      {'FR', 		1,'cm30VehicleWheelOut', -1, 'real', 'Sample'};
      {'RL', 		1,'cm30VehicleWheelOut', -1, 'real', 'Sample'};
      {'RR', 		1,'cm30VehicleWheelOut', -1, 'real', 'Sample'};
      {'RL2', 		1,'cm30VehicleWheelOut', -1, 'real', 'Sample'};
      {'RR2', 		1,'cm30VehicleWheelOut', -1, 'real', 'Sample'};
      {'FL2',	 	1,'cm30VehicleWheelOut', -1, 'real', 'Sample'};
      {'FR2', 		1,'cm30VehicleWheelOut', -1, 'real', 'Sample'};
    }
  }
  {
    'cm30BrakeWheelArray', {
      {'FL',	 	1,'cm30VehicleBrakeWheel', -1, 'real', 'Sample'};
      {'FR', 		1,'cm30VehicleBrakeWheel', -1, 'real', 'Sample'};
      {'RL', 		1,'cm30VehicleBrakeWheel', -1, 'real', 'Sample'};
      {'RR', 		1,'cm30VehicleBrakeWheel', -1, 'real', 'Sample'};
      {'RL2', 		1,'cm30VehicleBrakeWheel', -1, 'real', 'Sample'};
      {'RR2', 		1,'cm30VehicleBrakeWheel', -1, 'real', 'Sample'};
      {'FL2',	 	1,'cm30VehicleBrakeWheel', -1, 'real', 'Sample'};
      {'FR2', 		1,'cm30VehicleBrakeWheel', -1, 'real', 'Sample'};
    }
  }
  {
    'cm30PTWheelArray', {
      {'FL',	 	1,'cm30VehiclePTWheel', -1, 'real', 'Sample'};
      {'FR', 		1,'cm30VehiclePTWheel', -1, 'real', 'Sample'};
      {'RL', 		1,'cm30VehiclePTWheel', -1, 'real', 'Sample'};
      {'RR', 		1,'cm30VehiclePTWheel', -1, 'real', 'Sample'};
      {'RL2', 		1,'cm30VehiclePTWheel', -1, 'real', 'Sample'};
      {'RR2', 		1,'cm30VehiclePTWheel', -1, 'real', 'Sample'};
      {'FL2',	 	1,'cm30VehiclePTWheel', -1, 'real', 'Sample'};
      {'FR2', 		1,'cm30VehiclePTWheel', -1, 'real', 'Sample'};
    }
  }
  {
    'cm30WheelArray', {
      {'FL',	 	1,'double', -1, 'real', 'Sample'};
      {'FR', 		1,'double', -1, 'real', 'Sample'};
      {'RL', 		1,'double', -1, 'real', 'Sample'};
      {'RR', 		1,'double', -1, 'real', 'Sample'};
      {'RL2', 		1,'double', -1, 'real', 'Sample'};
      {'RR2', 		1,'double', -1, 'real', 'Sample'};
      {'FL2',	 	1,'double', -1, 'real', 'Sample'};
      {'FR2', 		1,'double', -1, 'real', 'Sample'};
    }
  }
})
end
Simulink.Bus.cellToObject({
% Input Bus
  {
    'cm30UserSignal', {
      {'s0',		1, 'double', -1, 'real', 'Sample'};
      {'s1',		1, 'double', -1, 'real', 'Sample'};
      {'s2',		1, 'double', -1, 'real', 'Sample'};
      {'s3',		1, 'double', -1, 'real', 'Sample'};
      {'s4',		1, 'double', -1, 'real', 'Sample'};
    }
  }
  {
    'cm30Steering', {
      { 'Ang',		1, 'double', -1, 'real', 'Sample' };
      { 'AngVel',	1, 'double', -1, 'real', 'Sample' };
      { 'AngAcc',	1, 'double', -1, 'real', 'Sample' };
      { 'Trq',		1, 'double', -1, 'real', 'Sample' };
      { 'SteerByTrq',	1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30Rider', {
      { 'RollAng',	1, 'double', -1, 'real', 'Sample' };
      { 'RollAngVel',	1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30Lights', {
      { 'LowBeam',	1, 'double', -1, 'real', 'Sample' };
      { 'HighBeam',	1, 'double', -1, 'real', 'Sample' };
      { 'Daytime',	1, 'double', -1, 'real', 'Sample' };
      { 'ParkL',	1, 'double', -1, 'real', 'Sample' };
      { 'ParkR',	1, 'double', -1, 'real', 'Sample' };
      { 'IndL',		1, 'double', -1, 'real', 'Sample' };
      { 'IndR',		1, 'double', -1, 'real', 'Sample' };
      { 'FogFrontL',	1, 'double', -1, 'real', 'Sample' };
      { 'FogFrontR',	1, 'double', -1, 'real', 'Sample' };
      { 'FogRear',	1, 'double', -1, 'real', 'Sample' };
      { 'Brake',	1, 'double', -1, 'real', 'Sample' };
      { 'Reverse',	1, 'double', -1, 'real', 'Sample' };
      { 'bm',		1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30VehicleControl', {
      { 'SST',			1, 'double', -1, 'real', 'Sample' };
      { 'Key',			1, 'double', -1, 'real', 'Sample' };
      { 'UserSignal',		1, 'cm30UserSignal', -1, 'real', 'Sample' };

      { 'GearNo',		1, 'double', -1, 'real', 'Sample' };
      { 'SelectorCtrl',		1, 'double', -1, 'real', 'Sample' };

      { 'Brake',		1, 'double', -1, 'real', 'Sample' };
      { 'BrakePark',		1, 'double', -1, 'real', 'Sample' };
      { 'BrakeLever',		1, 'double', -1, 'real', 'Sample' };
      { 'Clutch',		1, 'double', -1, 'real', 'Sample' };
      { 'Gas',			1, 'double', -1, 'real', 'Sample' };

      { 'Steering',		1, 'cm30Steering', -1, 'real', 'Sample' };
      { 'Rider',		1, 'cm30Rider', -1, 'real', 'Sample' };
      { 'Lights',		1, 'cm30Lights', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30xyz', {
      { 'x',	1, 'double', -1, 'real', 'Sample' };
      { 'y',	1, 'double', -1, 'real', 'Sample' };
      { 'z',	1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30xy', {
      { 'x',	1, 'double', -1, 'real', 'Sample' };
      { 'y',	1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30VehicleWheelIn', {
      { 'Trq_Brake',		1, 'double', -1, 'real', 'Sample' };
      { 'Trq_B2WC',		1, 'double', -1, 'real', 'Sample' };
      { 'rot',			1, 'double', -1, 'real', 'Sample' };
      { 'rotv',			1, 'double', -1, 'real', 'Sample' };
      { 'Trq_DL2WC',		1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30Vehicle', {
      { 'Wheel',		1, 'cm30WheelInArray', -1, 'real', 'Sample' };
      { 'Trq_DL2Bdy1',		1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'Trq_DL2Bdy1B',		1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'Trq_DL2BdyEng',	1, 'cm30xy', -1, 'real', 'Sample' };
      { 'Wind_vel_0',		1, 'cm30xyz', -1, 'real', 'Sample' };
    }
  }
  {
    'cmVehicle_CarIn', {
      { 'VhclCtrl',	1, 'cm30VehicleControl', -1, 'real', 'Sample' };
      { 'Vhcl',		1, 'cm30Vehicle', -1, 'real', 'Sample' };
    }
  }
% Output Bus
  {
    'cm30VehicleSteering', {
      { 'Ang',		1, 'double', -1, 'real', 'Sample' };
      { 'AngVel',	1, 'double', -1, 'real', 'Sample' };
      { 'AngAcc',	1, 'double', -1, 'real', 'Sample' };
      { 'Trq',		1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30VehicleMotion', {
      { 'sRoad',	1, 'double', -1, 'real', 'Sample' };
      { 'tRoad',	1, 'double', -1, 'real', 'Sample' };
      { 'Distance',	1, 'double', -1, 'real', 'Sample' };
      { 'v',		1, 'double', -1, 'real', 'Sample' };
      { 'Yaw',		1, 'double', -1, 'real', 'Sample' };
      { 'YawRate',	1, 'double', -1, 'real', 'Sample' };
      { 'YawAcc',	1, 'double', -1, 'real', 'Sample' };
      { 'Roll',		1, 'double', -1, 'real', 'Sample' };
      { 'RollVel',	1, 'double', -1, 'real', 'Sample' };
      { 'RollAcc',	1, 'double', -1, 'real', 'Sample' };
      { 'Pitch',	1, 'double', -1, 'real', 'Sample' };
      { 'PitchVel',	1, 'double', -1, 'real', 'Sample' };
      { 'PitchAcc',	1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30Hitch', {
      { 't_0',		1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'v_0',		1, 'cm30xyz', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30VehiclePoI', {
      { 'Pos',		1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'Vel',		1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'Acc',		1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'GCS_Long',	1, 'double', -1, 'real', 'Sample' };
      { 'GCS_Lat',	1, 'double', -1, 'real', 'Sample' };
      { 'GCS_Elev',	1, 'double', -1, 'real', 'Sample' };
      { 'Vel_1',	1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'Acc_1',	1, 'cm30xyz', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30VehicleWheelOut', {
      { 't',			1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'r_zxy',		1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'LongSlip',		1, 'double', -1, 'real', 'Sample' };
      { 'SideSlip',		1, 'double', -1, 'real', 'Sample' };
      { 'Fx',			1, 'double', -1, 'real', 'Sample' };
      { 'Fy',			1, 'double', -1, 'real', 'Sample' };
      { 'Fz',			1, 'double', -1, 'real', 'Sample' };
      { 'FxTwin',		1, 'double', -1, 'real', 'Sample' };
      { 'FyTwin',		1, 'double', -1, 'real', 'Sample' };
      { 'FzTwin',		1, 'double', -1, 'real', 'Sample' };
      { 'Trq_T2W',		1, 'double', -1, 'real', 'Sample' };
      { 'Trq_WhlBearing',	1, 'double', -1, 'real', 'Sample' };
      { 'vBelt',		1, 'double', -1, 'real', 'Sample' };
      { 'Trq_BrakeReg',		1, 'double', -1, 'real', 'Sample' };
      { 'Trq_BrakeReg_max',	1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30Matrix', {
      { 'xx',	1, 'double', -1, 'real', 'Sample' };
      { 'xy',	1, 'double', -1, 'real', 'Sample' };
      { 'xz',	1, 'double', -1, 'real', 'Sample' };
      { 'yx',	1, 'double', -1, 'real', 'Sample' };
      { 'yy',	1, 'double', -1, 'real', 'Sample' };
      { 'yz',	1, 'double', -1, 'real', 'Sample' };
      { 'zx',	1, 'double', -1, 'real', 'Sample' };
      { 'zy',	1, 'double', -1, 'real', 'Sample' };
      { 'zz',	1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30VehicleFr1A', {
      { 't_0',		1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'v_0',		1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'a_0',		1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'omega_0',	1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'alpha_0',	1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'Tr2Fr0',	1, 'cm30Matrix', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30VehicleBrakeWheel', {
      { 'Trq_WB',		1, 'double', -1, 'real', 'Sample' };
      { 'Trq_PB',		1, 'double', -1, 'real', 'Sample' };
      { 'Trq_BrakeReg_trg',	1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30VehiclePTWheel', {
      { 'Trq_B2W',	1, 'double', -1, 'real', 'Sample' };
      { 'rot',		1, 'double', -1, 'real', 'Sample' };
      { 'rotv',		1, 'double', -1, 'real', 'Sample' };
      { 'Trq_Supp2WC',	1, 'double', -1, 'real', 'Sample' };
      { 'Trq_BrakeReg',	1, 'double', -1, 'real', 'Sample' };
      { 'Trq_BrakeReg_max',	1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30VehiclePT', {
      { 'Trq_Supp2Bdy1',	1,'cm30xyz', -1, 'real', 'Sample' };
      { 'Trq_Supp2Bdy1B',	1,'cm30xyz', -1, 'real', 'Sample' };
      { 'Trq_Supp2BdyEng',	1, 'cm30xy', -1, 'real', 'Sample' };
      { 'Ignition',		1, 'double', -1, 'real', 'Sample' };
      { 'OperationState',	1, 'double', -1, 'real', 'Sample' };
      { 'OperationError',	1, 'double', -1, 'real', 'Sample' };
      { 'Engine_rotv',		1, 'double', -1, 'real', 'Sample' };
      { 'GearBox_GearNo',	1, 'double', -1, 'real', 'Sample' };
      { 'DL_iDiff_mean',	1, 'double', -1, 'real', 'Sample' };
      { 'Wheel',		1, 'cm30PTWheelArray', -1, 'real', 'Sample' };
    }
  }
  {
    'cmVehicle_CarOut', {
      { 'Steering',	1, 'cm30VehicleSteering', -1, 'real', 'Sample' };
      { 'Motion',	1, 'cm30VehicleMotion', -1, 'real', 'Sample' };
      { 'Hitch',	1, 'cm30Hitch', -1, 'real', 'Sample' };
      { 'PoI',		1, 'cm30VehiclePoI', -1, 'real', 'Sample' };
      { 'WheelOut',	1, 'cm30WheelOutArray', -1, 'real', 'Sample' };
      { 'Fr1A',		1, 'cm30VehicleFr1A', -1, 'real', 'Sample' };
      { 'Fr1B_r_xy',	1, 'cm30xy', -1, 'real', 'Sample' };
      { 'Brake',	1, 'cm30BrakeWheelArray', -1, 'real', 'Sample' };
      { 'PT',		1, 'cm30VehiclePT', -1, 'real', 'Sample' };
      { 'AeroMarkerPos',1, 'cm30xyz', -1, 'real', 'Sample' };
    }
  }
% Cfg Input Bus
  {
    'cm30Inertia', {
      { 'Ixx',	1, 'double', -1, 'real', 'Sample' };
      { 'Iyy',	1, 'double', -1, 'real', 'Sample' };
      { 'Izz',	1, 'double', -1, 'real', 'Sample' };
      { 'Ixy',	1, 'double', -1, 'real', 'Sample' };
      { 'Ixz',	1, 'double', -1, 'real', 'Sample' };
      { 'Iyz',	1, 'double', -1, 'real', 'Sample' };
      { 'Iyx',	1, 'double', -1, 'real', 'Sample' };
      { 'Izx',	1, 'double', -1, 'real', 'Sample' };
      { 'Izy',	1, 'double', -1, 'real', 'Sample' };
    }
  }
  {
    'cm30FGears', {
      {'Gear1',		1,'double', -1, 'real', 'Sample'};
      {'Gear2',		1,'double', -1, 'real', 'Sample'};
      {'Gear3',		1,'double', -1, 'real', 'Sample'};
      {'Gear4',		1,'double', -1, 'real', 'Sample'};
      {'Gear5',		1,'double', -1, 'real', 'Sample'};
      {'Gear6',		1,'double', -1, 'real', 'Sample'};
      {'Gear7',		1,'double', -1, 'real', 'Sample'};
      {'Gear8',		1,'double', -1, 'real', 'Sample'};
      {'Gear9',		1,'double', -1, 'real', 'Sample'};
      {'Gear10',	1,'double', -1, 'real', 'Sample'};
      {'Gear11',	1,'double', -1, 'real', 'Sample'};
      {'Gear12',	1,'double', -1, 'real', 'Sample'};
      {'Gear13',	1,'double', -1, 'real', 'Sample'};
      {'Gear14',	1,'double', -1, 'real', 'Sample'};
      {'Gear15',	1,'double', -1, 'real', 'Sample'};
      {'Gear16',	1,'double', -1, 'real', 'Sample'};
    }
  }
  {
    'cm30BGears', {
      {'Gear1',		1,'double', -1, 'real', 'Sample'};
      {'Gear2',		1,'double', -1, 'real', 'Sample'};
      {'Gear3',		1,'double', -1, 'real', 'Sample'};
      {'Gear4',		1,'double', -1, 'real', 'Sample'};
    }
  }
  {
    'cmVehicle_CarCfgIn', {
      { 'MinStartPos',		1, 'double', -1, 'real', 'Sample' };

      { 'PoI_Pos_1',		1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'PoI2AxleFront',	1, 'double', -1, 'real', 'Sample' };
      { 'PoI2AxleRear',		1, 'double', -1, 'real', 'Sample' };
      { 'PoI2CarEndLeft',	1, 'double', -1, 'real', 'Sample' };
      { 'PoI2CarEndRight',	1, 'double', -1, 'real', 'Sample' };
      { 'CoG2AxleFront',	1, 'double', -1, 'real', 'Sample' };
      { 'CoG2AxleRear',		1, 'double', -1, 'real', 'Sample' };
      { 'WheelBase',		1, 'double', -1, 'real', 'Sample' };
      { 'TrackWidth',		1, 'double', -1, 'real', 'Sample' };
      { 'OverhangFront',	1, 'double', -1, 'real', 'Sample' };
      { 'OverhangRear',		1, 'double', -1, 'real', 'Sample' };

      { 'MassTotal',		1, 'double', -1, 'real', 'Sample' };
      { 'Inertia',		1, 'cm30Inertia', -1, 'real', 'Sample' };

      { 'nWheels',		1, 'double', -1, 'real', 'Sample' };
      { 'WhlRadius',		1, 'double', -1, 'real', 'Sample' };
      { 'iDiff',		1, 'double', -1, 'real', 'Sample' };
      { 'nFGears',		1, 'double', -1, 'real', 'Sample' };
      { 'nBGears',		1, 'double', -1, 'real', 'Sample' };
      { 'iFGear',		1, 'cm30FGears', -1, 'real', 'Sample' };
      { 'iBGear',		1, 'cm30BGears', -1, 'real', 'Sample' };
      { 'GBKind',		1, 'double', -1, 'real', 'Sample' };
      { 'PTKind',		1, 'double', -1, 'real', 'Sample' };
      { 'SteerByTorque',	1, 'double', -1, 'real', 'Sample' };
      { 'StartEngineWithSST',	1, 'double', -1, 'real', 'Sample' };

      { 'Whl_Iyy',		1, 'cm30WheelArray', -1, 'real', 'Sample' };
      { 'Engine_rotv_Idle',	1, 'double', -1, 'real', 'Sample' };
      { 'Engine_rotv_Max',	1, 'double', -1, 'real', 'Sample' };
      { 'Velocity_Max',		1, 'double', -1, 'real', 'Sample' };
      { 'Aero_Frcx',		1, 'double', -1, 'real', 'Sample' };

      { 'Whl_rotv',		1, 'cm30WheelArray', -1, 'real', 'Sample' };
      { 'GearBox_rotv_out',	1, 'double', -1, 'real', 'Sample' };
      { 'Engine_rotv',		1, 'double', -1, 'real', 'Sample' };
      { 'Roll',			1, 'double', -1, 'real', 'Sample' };
      { 'Yaw',			1, 'double', -1, 'real', 'Sample' };
      { 'Pitch',		1, 'double', -1, 'real', 'Sample' };
      { 'Fr1_Pos',		1, 'cm30xyz', -1, 'real', 'Sample' };
      { 'Distance',		1, 'double', -1, 'real', 'Sample' };
    }
  }
});
