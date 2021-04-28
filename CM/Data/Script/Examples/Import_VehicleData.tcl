##
## Import Vehicle Data
## CarMaker 7.0 ScriptControl Example - IPG Automotive GmbH (www.ipg-automotive.com)
##
## Syntax: Car::Import <FileName> <DataList>
##
## Known Options for DataList:
##	Model
##	Masses
##
##	Spring.Front
##	Spring.Rear
##	Damper.Front
##	Damper.Rear
##	Buffer.Front
##	Buffer.Rear
##	Stabilizer.Front
##	Stabilizer.Rear
##	Kinematics.Front
##	Kinematics.Rear
##
##	ExtForces
##	Steering
##	Brake
##
##	Engine
##	Clutch
##	GearBox
##	DiffFront
##	DiffRear
##	DiffCenter
##	DiffHangOn
##
##	Aerodynamics
##	Misc
##
## $Id: Import_VehicleData.tcl,v 1.6 2018/04/17 12:36:26 kh Exp $

set lst {Steering Brake Misc Spring.Front Pushrod}

set FName Examples/Demo_Ford_Focus
set ierr [Car::Import $FName $lst]
Log "# 1: $FName -> $ierr\n"

set FName Examples/Demo_Jaguar_XType
set ierr [Car::Import $FName $lst]
Log "# 2: $FName -> $ierr\n"

set FName Examples/Demo_Lexus_RX400h
set ierr [Car::Import $FName $lst]
Log "# 3: $FName -> $ierr\n"

set lst {Steering Brake Aerodynamics Misc Spring.Front Damper.Rear}

set FName Examples/Demo_Porsche_911
set ierr [Car::Import $FName $lst]
Log "# 4: $FName -> $ierr\n"

set FName Examples/DemoCar
set ierr [Car::Import $FName]
Log "# 5: $FName -> $ierr\n"

