##
## RunScripts.tcl 
## CarMaker 7.0 ScriptControl Example - IPG Automotive GmbH (www.ipg-automotive.com)
##
## $Id: RunScripts.tcl,v 1.5 2018/04/17 12:36:26 kh Exp $


# A script can specified with a path relative to current script
RunScript StartStop.tcl
RunScript WaitForCondition.tcl
RunScript Math.tcl

# Absolute paths can also be used, but should be avoided.
# Example:
# RunScript /home/hil/Scripts/StartStop.tcl


