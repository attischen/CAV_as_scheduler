#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2022 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26

from __future__ import absolute_import
from __future__ import print_function

from MILPSubproblem import solveDC
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import optparse
import random
import math
import copy
import pandas as pd
# we need to import python modules from the $SUMO_HOME/tools directory

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
STEPLENGTH = 1#time for a step

def generate_routefile(HVratio,meanInterval):
    #random.seed(19)  # make tests reproducible
    N = 100  # number of time steps
    # demand per second from different directions
    pWE = 1/meanInterval  * STEPLENGTH
    pEW = 1/meanInterval * STEPLENGTH
    pNS = 1/meanInterval * STEPLENGTH
    pSN = 1/meanInterval * STEPLENGTH

    with open("data/cross.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="CAV" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" \
guiShape="passenger"/>
        <vType id="HV" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" \
guiShape="passenger"/>

        <route id="right" edges="51o 1i 2o 52i" />
        <route id="left" edges="52o 2i 1o 51i" />
        <route id="up" edges="53o 3i 4o 54i" />
        <route id="down" edges="54o 4i 3o 53i" />""", file=routes)
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < pWE:
                if random.uniform(0,1) < HVratio:
                    print('    <vehicle id="right_%i" type="HV"  route="right" depart="%i" />' % (
                    vehNr, i), file=routes)
                    vehNr += 1
                else:
                    print('    <vehicle id="right_%i" type="CAV" route="right" depart="%i" color="0,0,1"/>' % (
                    vehNr, i), file=routes)
                    vehNr += 1

                
            if random.uniform(0, 1) < pEW:
                if random.uniform(0,1) < HVratio:
                    print('    <vehicle id="left_%i" type="HV"  route="left" depart="%i" />' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                else:
                    print('    <vehicle id="left_%i" type="CAV" route="left" depart="%i" color="0,0,1"/>' % (
                        vehNr, i), file=routes)
                    vehNr += 1

            if random.uniform(0, 1) < pNS:
                if random.uniform(0,1) < HVratio:
                    print('    <vehicle id="down_%i" type="HV"  route="down" depart="%i" />' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                else:
                    print('    <vehicle id="down_%i" type="CAV" route="down" depart="%i" color="0,0,1"/>' % (
                        vehNr, i), file=routes)
                    vehNr += 1

            if random.uniform(0, 1) < pSN:
                if random.uniform(0,1) < HVratio:
                    print('    <vehicle id="up_%i" type="HV"  route="up" depart="%i" />' % (
                        vehNr, i), file=routes)
                    vehNr += 1
                else:
                    print('    <vehicle id="up_%i" type="CAV" route="up" depart="%i" color="0,0,1"/>' % (
                        vehNr, i), file=routes)
                    vehNr += 1

        print("</routes>", file=routes)

# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/>
#        <phase duration="6"  state="yryr"/>
#        <phase duration="31" state="rGrG"/>
#        <phase duration="6"  state="ryry"/>
#    </tlLogic>

class IntersectionManager():
    def __init__(self,incomingLanes,outboundLanes,sensorRange,interval,schedulePower,method):
        self.sensorRange = sensorRange
        self.scheduleInterval = interval
        self.expectedArrivalTime = {}
        self.vInRange = {}  #stores by each lane 
        self.position =  (510,510)
        self.incomingLanes = incomingLanes
        self.outboundLanes = outboundLanes 
        self.scheduledPasstime = None
        self.passingOrder = {}
        self.nonConflictingLanes = [(0,1),(1,0),(2,3),(3,2)]
        self.blockDistance = 10
        self.intersectionRadius = 5
        self.blockList = []
        self.stopRadius = 15
        self.P1 = 1
        self.P2 = 3
        self.stoppedVehicle = []
        self.existsHV = None
        self.allowedPassTime = 0
        self.schedulePower=schedulePower
        self.method=method

    def vDistance(self,v):
        pos = traci.vehicle.getPosition(v)
        x,y = pos[0] - self.position[0], pos[1] - self.position[1]
        return math.sqrt(x**2 + y**2)

    def updateVInRange(self):
        vInRange = {}
        for lane in self.incomingLanes:
            vInLane = []
            vList = traci.edge.getLastStepVehicleIDs(lane)
            for v in vList:
                if self.vDistance(v) <= self.sensorRange:
                    vInLane.append(v)
            vInRange[lane] = vInLane[::-1] #head vehicle at 0 
        self.vInRange = vInRange 

    def updatePassedVehicle(self):
        for lane in self.outboundLanes:
            vList = traci.edge.getLastStepVehicleIDs(lane)
            for v in vList:
                if v in self.blockList:
                    self.blockList.remove(v)
                    traci.vehicle.setSpeed(v,-1) # unset
                    traci.vehicle.setColor(v,(0,0,255,1))

    def updateExceptedArrivalTime(self):
        expectedArrivalTime = {}
        for lane in self.vInRange:
            vList = self.vInRange[lane]
            for v in vList:
                expectedArrivalTime[v] = self.vDistance(v) / traci.vehicle.getAllowedSpeed(v)

        self.expectedArrivalTime = expectedArrivalTime

    def schedule(self):
        # convert data format 
        arrivalTime = []
        HV = []

        for lane in self.vInRange:
            vlist = self.vInRange[lane]
            arrival = []
            hv = []
            for i,v in enumerate(vlist):
                if i >= self.schedulePower:
                    break
                arrival.append(self.expectedArrivalTime[v])
                hv.append(1 if traci.vehicle.getTypeID(v) == "HV" else 0) 

            arrivalTime.append(arrival)
            HV.append(hv)
        passtime,_,_ = self.method(arrivalTime,HV,[(0,1),(2,3)],self.P1,self.P2)
        
        if passtime is not None:
            passingOrder = {}
            for l,lane in enumerate(self.vInRange):
                vlist = self.vInRange[lane]
                for i,v in enumerate(vlist):
                    if i >= self.schedulePower:
                        break
                    passingOrder[v] = sum([1 if t <  passtime[l][i] else 0 for lane in passtime for t in lane])
            self.passingOrder = passingOrder
        #print(self.passingOrder)

    def stopBeforePass(self):         
        heads = []
        for lane in self.vInRange:
            vList = self.vInRange[lane]
            heads.append(vList[0] if len(vList) > 0 else None)
        existsHV = False
        headCAV = []
        for l,head in enumerate(heads):
            if head != None:
                if traci.vehicle.getTypeID(head) == "HV":
                    self.existsHV = True
                else:
                    headCAV.append(head)

        if self.existsHV:
            for v in heads:
                if v != None:
                    if v not in self.stoppedVehicle and self.vDistance(v) < self.stopRadius:
                        traci.vehicle.setSpeed(v,0)
                        self.stoppedVehicle.append(v)  

    def startPass(self,currentTime,schedule=True):
        for v in self.stoppedVehicle:
            if currentTime >= self.allowedPassTime:
                if len(self.passingOrder) == 0 or (traci.vehicle.getTypeID(v) == "HV") or (v in self.passingOrder and self.passingOrder[v] == 0):
                    traci.vehicle.setSpeed(v,-1)
                    self.allowedPassTime = max(self.allowedPassTime,currentTime + (self.P2 if self.existsHV else self.P1))
                    break
        
        for v in self.stoppedVehicle:
            if self.vDistance(v) >= self.stopRadius:
                self.stoppedVehicle.remove(v)

    def executeBlocking(self):
        if self.passingOrder == None:
            return
        heads = []
        for lane in self.vInRange:
            vList = self.vInRange[lane]
            heads.append(vList[0] if len(vList) > 0 else None)

        for l,head in enumerate(heads):
            if head != None:
                if traci.vehicle.getTypeID(head) == "CAV":
                    if head not in self.blockList and self.decideBlock(l,head,heads):
                        #print("block " + head)
                        traci.vehicle.setSpeed(head,0)
                        #traci.vehicle.setColor(head,(255,0,0,1))
                        self.blockList.append(head)
                        print(self.blockList)
                    elif head in self.blockList:
                        self.blockList.remove(head)
                        traci.vehicle.setSpeed(head,-1) # unset
                        #traci.vehicle.setColor(head,(0,0,255,1))


    def decideBlock(self,blockLane,blockingVehicle,heads):
        #heads is a dict
        for l,head in enumerate(heads):
            if blockLane == l or head not in self.passingOrder or blockingVehicle not in self.passingOrder:
                continue
            if self.passingOrder[head] == 0 and (l,blockLane) not in self.nonConflictingLanes:
                if self.vDistance(blockingVehicle) < self.blockDistance and self.vDistance(blockingVehicle) > self.intersectionRadius:
                    return True
        return False




class Analysis():
    def __init__ (self,outboundLanes):
        self.waitTime = {} 
        self.passedVehicle = []
        self.outboundLanes = outboundLanes
        self.hv = {}
    def updateWaitTime(self):
        for lane in self.outboundLanes:
            vLeaving = traci.edge.getLastStepVehicleIDs(lane)
            for v in vLeaving:
                if v not in self.passedVehicle:
                    self.waitTime[v] = traci.vehicle.getAccumulatedWaitingTime(v)
                    self.passedVehicle.append(v)
                    self.hv[v] = True if  traci.vehicle.getTypeID(v) == "HV" else False
                    #print(v,self.waitTime[v])
    def getAvgWaitTime(self):
        #print(self.waitTime)
        totalWTime = 0
        cavWaitTime = 0
        hvWaitTime = 0
        hvCount = 0
        cavCount = 0
        for v in self.passedVehicle:
            totalWTime += self.waitTime[v]
            if self.hv[v]:
                hvWaitTime += self.waitTime[v]
                hvCount += 1
            else:
                cavWaitTime += self.waitTime[v]
                cavCount += 1
        totalWTime = 0 if len(self.waitTime) == 0 else totalWTime/len(self.waitTime)
        cavWaitTime = 0 if cavCount == 0 else cavWaitTime / cavCount 
        hvWaitTime = 0 if hvCount == 0 else hvWaitTime / hvCount
        return totalWTime,cavWaitTime,hvWaitTime 

    def getAvgTotalPassTime(self,IM):
        #print(self.waitTime)
        totalPTime = 0
        for v in IM.passTimes:
            totalPTime += IM.passTimes[v] - IM.enterTimes[v]
        return 0 if len(IM.passTimes) == 0 else totalPTime/len(IM.passTimes)


def run(schedulePower,schedule=True,method=solveDC):
    """execute the TraCI control loop"""
    step = 0
    sensorRange = 500
    scheduleInterval = 1 # seconds
    lastScheduleTime = -math.inf
    incomingLanes = ["1i","2i","3i","4i"]
    outboundLanes = ["2o","1o","4o","3o"]
    IM = IntersectionManager(incomingLanes,outboundLanes,sensorRange,scheduleInterval,schedulePower,method) 
    analysis = Analysis(outboundLanes)

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        #print(traci.edge.getIDList())
        IM.updatePassedVehicle()
        IM.updateVInRange()
        IM.updateExceptedArrivalTime()
        if schedule and step * STEPLENGTH - lastScheduleTime >= scheduleInterval:
            lastScheduleTime = step * STEPLENGTH
            IM.schedule()
        IM.stopBeforePass()
        #IM.executeBlocking()
        IM.startPass(step * STEPLENGTH,schedule=schedule)

        analysis.updateWaitTime()
        step += 1
        sys.stdout.flush()

    traci.close()
    avgWaitTime,cavWaitTime,hvWaitTime = analysis.getAvgWaitTime()
    print(avgWaitTime)
    return avgWaitTime,cavWaitTime,hvWaitTime


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    #sumoBinary = "/mnt/c/Program Files (x86)/Eclipse/Sumo/bin/sumo-gui.exe"
    sumoBinary = "/mnt/c/Program Files (x86)/Eclipse/Sumo/bin/sumo.exe"
    meanInterval = 20
    #schedulePower 
    scheduleWait = []
    unscheduleWait =  []
    schedulePowers = [3]
    testCounts = 20
    for schedulePower in schedulePowers:
        scheduleTotal = []
        scheduleCAV = []
        scheduleHV = []
        unScheduleTotal = []
        unScheduleCAV = []
        unScheduleHV = []
        for HVratio in np.arange(0.0,1.1,0.1):
            avgTotal = 0
            avgCAV = 0
            avgHV = 0
            avgTotal2 = 0
            avgCAV2 = 0
            avgHV2 = 0
            for test in range(testCounts):
                generate_routefile(HVratio,meanInterval)
                traci.start([sumoBinary, "-c", "./data/cross.sumocfg","--tripinfo-output", "tripinfo.xml"])
                totalWait,cavWait,hvWait = run(schedulePower,schedule = True)
                avgTotal += totalWait
                avgCAV +=  cavWait
                avgHV += hvWait
                
                traci.start([sumoBinary, "-c", "./data/cross.sumocfg","--tripinfo-output", "tripinfo.xml"])
                totalWait,cavWait,hvWait = run(schedulePower,schedule = False)
                avgTotal2 += totalWait
                avgCAV2 +=  cavWait
                avgHV2 += hvWait

            scheduleTotal.append(avgTotal/testCounts)
            scheduleCAV.append(avgCAV/testCounts)
            scheduleHV.append(avgHV/testCounts)

            unScheduleTotal.append(avgTotal2/testCounts)
            unScheduleCAV.append(avgCAV2/testCounts)
            unScheduleHV.append(avgHV2/testCounts)
        
        plt.figure(1)
        plt.plot(np.arange(0.0,1.1,0.1),scheduleTotal,label = "Scheduled average wait time")
        plt.plot(np.arange(0.0,1.1,0.1),unScheduleTotal,label = "Unscheduled average wait time")
        plt.figure(2)
        plt.plot(np.arange(0.0,1.1,0.1),scheduleCAV,label = "Scheduled CAV wait time")
        plt.plot(np.arange(0.0,1.1,0.1),scheduleHV,label = "Scheduled HV wait time")
        plt.plot(np.arange(0.0,1.1,0.1),unScheduleCAV,label = "Unscheduled CAV wait time")
        plt.plot(np.arange(0.0,1.1,0.1),unScheduleHV,label = "Unscheduled HV wait time")

   
    plt.figure(1)
    plt.xlabel('HV Ratio')
    plt.ylabel('Cost(s)')
    plt.legend()
    fig = plt.gcf() 
    fig.savefig('SumoavgWaitTime'+ str(meanInterval) + '.svg')

    plt.figure(2)
    plt.xlabel('HV Ratio')
    plt.ylabel('Cost(s)')
    plt.legend()
    fig = plt.gcf() 
    fig.savefig('SumoWaitTimeHVvsCAV'+ str(meanInterval) + '.svg')

    df = pd.DataFrame({
        'Scheduled': scheduleTotal,
        'Unscheduled': unScheduleTotal,
        'Cost ratio': [ round(max(scheduleTotal[i],0.01)/ max(unScheduleTotal[i],0.01),3) for i in range(len(scheduleTotal))]
        },index = np.arange(0,1.1,0.1))
    df.index.name = "HV ratio"
    df.to_csv("SUMO_nosignal.csv")

