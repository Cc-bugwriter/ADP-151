from bs4 import BeautifulSoup
import re
import numpy as np
import matplotlib.pyplot as plt


class mapParser():

    def __init__(self, mapFile):

        #mapFile is string, containing path to the xodr map file, like ../../map.xodr
        self.mapFile = mapFile

        #initializing parametering like minimium step interval or road width
        self.parameterSetting()

    def parameterSetting(self):
        self.road_width = 3.5
        self.point_min_interval = 0.01

        print("parameterSetting is done!")

    def pathReading(self, pathIndexList, pathTypeList, pathDirectList):

        """
            
        """

        #ensure the number of elements of these three list is same
        #else will abort the processing
        inputList = [len(pathIndexList), len(
            pathTypeList), len(pathDirectList)]

        if(len(np.unique(inputList)) != 1):
            print("error!")

        self.road_num = len(pathIndexList)


        #pathIndexList is a list, containing id of each road in a given order
        #the order is set up by a given global path planner, like starting, .... , destination.
        self.pathIndexList = pathIndexList

        #pathTypeList refers to the type definition of the road
        #in OpenDRIVE, as we simplifed, the road divided into two type:
        # road out of the junciton -> type 2
        # road inside the junction -> type 1
        self.pathTypeList = pathTypeList

        #pathDirectList describes the forward direction of the road 
        #in OpenDRIVE,
        self.pathDirectList = pathDirectList

        print("pathReading is done!")

    def mapReading(self):

        with open(self.mapFile) as f:
            #strip whitespace or other useless seperator in the file#
            #intend to avoid errors#
            self.map = f.read().strip()
        
        #applying a XML parser with BS4
        self.map = BeautifulSoup(self.map, "lxml")

        #applying a CSS selector to get all the node with tag road in the map
        self.roadListAll = self.map.find_all(name="road")

        #select the nodes with the id we expected
        self.roadList = [self.roadListAll[x-1] for x in self.pathIndexList]

        #initializing a discrete point list (x,y) of the centerline of the road
        self.refCenter=pointList(self.road_num)

        #calculating iscrete point list (x,y) of the centerline of the road
        self.curveProcessing(self.refCenter,0)

        #pointList has atrributes like cord_x_seperate,cord_x_seperate
        #these two are both list of list,like [[1,2],[3,4],[5,6]]
        #each childrenlist representing a part of a road,like line part or arc part
        #to simplify processing, it is neccssaty to merge them into two 1-D arrays.
        self.curveMerge(self.refCenter)

        print("mapReading is done!")

    def curveProcessing(self,point_List,offSet):
        
        point_min_interval=self.point_min_interval

        for roadIndex, roadItem in enumerate(self.roadList):

            geoList = roadItem.find_all(name="geometry")

            for geoitemIndex, geoItem in enumerate(geoList):

                childrenItem = list(geoItem.children)[1]

                attrs = geoItem.attrs

                ## x0 y0 are begin of the points
                x0 = float(attrs["x"])
                y0 = float(attrs["y"])

                ## s is the length of this road section part
                s = float(attrs["length"])

                ## hdg is Slope of this straight line ;
                hdg = float(attrs["hdg"])

                if childrenItem.name == "line":

                    point_List.cord_x_seperate[roadIndex].append(x0+np.cos(hdg-np.pi/2)*offSet)
                    point_List.cord_y_seperate[roadIndex].append(y0+np.sin(hdg-np.pi/2)*offSet)


                    #to make ensure the point interval is what we expect(equals variable : point_min_interval)

                    [quotient,remainder]=divmod(s,point_min_interval)
                    quotient=int(quotient)

                    #s_space is a discrete list of s ; 
                    #s_space is used to make sure the the point interval is what we expect
                    # for example s= 10m point_min_interval=0.1m;
                    # s_space =[0 , 0.1 , 0.2 , 0.3 , 0.4 , ... , 9.9 , 10.0 ]
                    
                    #we check the python offical doc: the calculation of function numpy.linespace and function divmod maynot be accurate by minor step.
                    #it is necessary to check the last element/endPoint (quotient * point_min_interval) is large or smaller than s.
                    #if smaller , we should add s to tail of the list s_space;
                    #if biger, we should remove the last element of list s_space and add s to tail of the list s_space.

                    if (quotient*point_min_interval<s):
                        s_space=[point_min_interval*index for index in range(1,quotient+1)]
                        s_space.append(s)

                    else:
                        s_space=[point_min_interval*index for index in range(1,quotient+1)]
                        s_space.pop()
                        s_space.append(s)

                    for ss in s_space:

                        x1 = x0+ss*np.cos(hdg)
                        y1 = y0+ss*np.sin(hdg)

                        point_List.cord_x_seperate[roadIndex].append(x1+np.cos(hdg-np.pi/2)*offSet)
                        point_List.cord_y_seperate[roadIndex].append(y1+np.sin(hdg-np.pi/2)*offSet)



                elif childrenItem.name == "poly3":

                    point_List.cord_x_seperate[roadIndex].append(x0+np.cos(hdg-np.pi/2)*offSet)
                    point_List.cord_y_seperate[roadIndex].append(y0+np.sin(hdg-np.pi/2)*offSet)

                    nextSegItem=geoList[geoitemIndex+1]

                    nextSegAttr=nextSegItem.attrs

                    x_netSeg=float(nextSegAttr["x"])
                    y_netSeg=float(nextSegAttr["y"])
                    hdg_nextSeg=float(nextSegAttr["hdg"])


                    vec_angle=np.arctan((y_netSeg-y0)/(x_netSeg-x0))

                    if vec_angle>0:
                        if y_netSeg-y0 < 0:
                            vec_angle=vec_angle-np.pi
                    elif vec_angle<0:
                        if y_netSeg-y0 > 0:
                            vec_angle=vec_angle+np.pi
                    elif vec_angle==0:
                        if x_netSeg-x0 < 0:
                            vec_angle=np.pi
                    
                    vec_beta=vec_angle-hdg
                    vec_length=np.sqrt(np.power(x_netSeg-x0,2)+np.power(y_netSeg-y0,2))

                    u_final=vec_length*np.cos(vec_beta)

                    ##to calulate the final U with next segment starting points;

                    #to make ensure the point interval is what we expect(equals variable : point_min_interval)

                    [quotient,remainder]=divmod(u_final,point_min_interval)
                    quotient=int(quotient)

                    #s_space is a discrete list of s ; 
                    #s_space is used to make sure the the point interval is what we expect
                    # for example s= 10m point_min_interval=0.1m;
                    # s_space =[0 , 0.1 , 0.2 , 0.3 , 0.4 , ... , 9.9 , 10.0 ]
                    
                    #we check the python offical doc: the calculation of function numpy.linespace and function divmod maynot be accurate by minor step.
                    #it is necessary to check the last element/endPoint (quotient * point_min_interval) is large or smaller than s.
                    #if smaller , we should add s to tail of the list s_space;
                    #if biger, we should remove the last element of list s_space and add s to tail of the list s_space.

                    if (quotient*point_min_interval<u_final):
                        s_space=[point_min_interval*index for index in range(1,quotient+1)]
                        s_space.append(u_final)

                    else:
                        s_space=[point_min_interval*index for index in range(1,quotient+1)]
                        s_space.pop()
                        s_space.append(u_final)

                    


                    ploy_attrs = childrenItem.attrs
                    ploy_a = float(ploy_attrs["a"])
                    ploy_b = float(ploy_attrs["b"])
                    ploy_c = float(ploy_attrs["c"])
                    ploy_d = float(ploy_attrs["d"])

                    for u in s_space:
                        v = ploy_a+ploy_b*u+ploy_c*u**2+ploy_d*u**3
                        beta = hdg+np.arctan(v/u)
                        l = np.sqrt(v**2+u**2)

                        x1=x0+l*np.cos(beta)
                        y1=y0+l*np.sin(beta)

                        point_List.cord_x_seperate[roadIndex].append(x1+np.cos(beta-np.pi/2)*offSet)
                        point_List.cord_y_seperate[roadIndex].append(y1+np.sin(beta-np.pi/2)*offSet)



                elif childrenItem.name == "arc":

                    point_List.cord_x_seperate[roadIndex].append(x0+np.cos(hdg-np.pi/2)*offSet)
                    point_List.cord_y_seperate[roadIndex].append(y0+np.sin(hdg-np.pi/2)*offSet)

                    curvature = float(childrenItem.attrs["curvature"])

                    radius = 1/np.abs(curvature)

                    beta = s*np.abs(curvature)

                    #to make ensure the point interval is what we expect(equals variable : point_min_interval)

                    beta_min_interval=point_min_interval*np.abs(curvature)

                    [quotient,remainder]=divmod(beta,beta_min_interval)
                    quotient=int(quotient)

                    #s_space is a discrete list of s ; 
                    #s_space is used to make sure the the point interval is what we expect
                    # for example s= 10m point_min_interval=0.1m;
                    # s_space =[0 , 0.1 , 0.2 , 0.3 , 0.4 , ... , 9.9 , 10.0 ]
                    
                    #we check the python offical doc: the calculation of function numpy.linespace and function divmod maynot be accurate by minor step.
                    #it is necessary to check the last element/endPoint (quotient * point_min_interval) is large or smaller than s.
                    #if smaller , we should add s to tail of the list s_space;
                    #if biger, we should remove the last element of list s_space and add s to tail of the list s_space.

                    if (quotient*beta_min_interval<beta):
                        beta_space=[beta_min_interval*index for index in range(1,quotient+1)]
                        beta_space.append(beta)

                    else:
                        beta_space=[beta_min_interval*index for index in range(1,quotient+1)]
                        beta_space.pop()
                        beta_space.append(beta)


                    if curvature > 0:
                        degree1 = hdg+np.pi/2

                        x_center = x0+radius*np.cos(degree1)
                        y_center = y0+radius*np.sin(degree1)

                        # delta_beta = np.linspace(0, beta, 50)
                        # delta_bet = delta_beta[1:]

                        degree2 = hdg+np.pi*3/2

                        for angel in beta_space:

                            x1=x_center+radius*np.cos(degree2+angel)
                            y1=y_center+radius*np.sin(degree2+angel)

                            point_List.cord_x_seperate[roadIndex].append(x1+np.cos(hdg-np.pi/2+angel)*offSet)
                            point_List.cord_y_seperate[roadIndex].append(y1+np.sin(hdg-np.pi/2+angel)*offSet)


                    else:
                        degree1 = hdg-np.pi/2

                        x_center = x0+radius*np.cos(degree1)
                        y_center = y0+radius*np.sin(degree1)

                        # delta_beta = np.linspace(0, beta, 50)
                        # delta_bet = delta_beta[1:]

                        degree2 = hdg-np.pi*3/2

                        for angel in beta_space:

                            x1=x_center+radius*np.cos(degree2-angel)
                            y1=y_center+radius*np.sin(degree2-angel)

                            point_List.cord_x_seperate[roadIndex].append(x1+np.cos(hdg-np.pi/2-angel)*offSet)
                            point_List.cord_y_seperate[roadIndex].append(y1+np.sin(hdg-np.pi/2-angel)*offSet)

    def curveMerge(self,point_List):
        print()
        for i in range(0,self.road_num):

            point_List.eleNumList[i]=len(point_List.cord_x_seperate[i])

            if self.pathTypeList[i]==2:
                if self.pathDirectList[i]==-1:

                    tmp_x=point_List.cord_x_seperate[i]
                    tmp_y=point_List.cord_y_seperate[i]

                elif self.pathDirectList[i]==1:

                    tmp_x=point_List.cord_x_seperate[i]
                    tmp_y=point_List.cord_y_seperate[i]
                    tmp_x.reverse()
                    tmp_y.reverse()

            elif self.pathTypeList[i]==1:
                tmp_x=point_List.cord_x_seperate[i]
                tmp_y=point_List.cord_y_seperate[i]

            point_List.cord_x_merged=point_List.cord_x_merged+tmp_x
            point_List.cord_y_merged=point_List.cord_y_merged+tmp_y
        
        point_List.eleNum=len(point_List.cord_y_merged)


        with open("test.txt","w") as f:

            for item in range(0,point_List.eleNum):

                f.write(str(point_List.cord_x_merged[item])+" "+str(point_List.cord_y_merged[item])+"\n")




class pointList:

    #pointList has atrributes like cord_x_seperate,cord_x_seperate
    #these two are both list of list,like [[1,2],[3,4],[5,6]]
    #each childrenlist representing a part of a road,like line part or arc part
    
    def __init__(self, roadNum):
        self.cord_x_seperate = [[] for x in range(roadNum)]
        self.cord_y_seperate = [[] for x in range(roadNum)]
        self.cord_x_merged=[]
        self.cord_y_merged=[]
        self.eleNum=0
        self.eleNumList=[[] for x in range(roadNum)]


    def plotCurve(self):
        plt.plot(self.cord_x_merged,self.cord_y_merged)
        plt.show()
    