from mapparser import *

pathList=[3,22,6,11,4,13,5,20,3]
pathType=[2,1,2,1,2,1,2,1,2]
pathDirect=[-1,-1,1,-1,-1,-1,1,-1,1]

map_Gries=mapParser("Griesheim_map_new.xodr")

map_Gries.pathReading(pathList,pathType,pathDirect)

map_Gries.mapReading()

map_Gries.refCenter.plotCurve()
