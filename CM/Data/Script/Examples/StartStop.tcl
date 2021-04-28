##
## StartStop.tcl 
## CarMaker 7.0 ScriptControl Example - IPG Automotive GmbH (www.ipg-automotive.com)
## 
## Load, start and stop test runs
## 
## $Id: StartStop.tcl,v 1.7 2018/11/13 12:11:56 kh Exp $

Log "* Run 15 seconds of Hockenheim"
LoadTestRun "Examples/VehicleDynamics/Handling/Racetrack_Hockenheim"
StartSim
WaitForStatus running
WaitForTime 15

StopSim
WaitForStatus idle 10000

Log "* Run 15 seconds of LaneChangeISO"
LoadTestRun "Examples/VehicleDynamics/Handling/LaneChange_ISO"
StartSim
WaitForStatus running
WaitForTime 15

StopSim

