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

from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import optparse
import random
import math
import copy
import pandas as pd


sys.path.append('../../../')
from CAV_as_scheduler.Gurobi.MILPBased import MILPBased

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary 
import traci   


def generate_routefile(HVratio,MEANINTERVAL):
    # demand per second from different directions
    pWE = 1/MEANINTERVAL * STEPLENGTH
    pEW = 1/MEANINTERVAL * STEPLENGTH
    pNS = 1/MEANINTERVAL * STEPLENGTH
    pSN = 1/MEANINTERVAL * STEPLENGTH

    with open("sumocfg/cross.rou.xml", "w") as routes:
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
        for i in range(TIMESTEPS):
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

class IntersectionManager():
    def __init__(self,incomingLanes,outboundLanes,sensorRange,interval,schedulePower,method=MILPBased):
        self.sensorRange = sensorRange 
        self.scheduleInterval = interval
        self.expectedArrivalTime = {}
        self.vInRange = {}  #vehicles within sensor range, stores by each lane 
        self.position =  (510,510) # position of intersection manager 
        self.incomingLanes = incomingLanes
        self.outboundLanes = outboundLanes 
        self.scheduledPasstime = None
        self.passingOrder = {}
        self.nonConflictingLanes = [(0,1),(1,0),(2,3),(3,2)]
        self.intersectionRadius = 5 
        self.stopRadius = 15
        self.G1 = G1
        self.G2 = G2
        self.stoppedVehicle = []
        self.existsHV = None # if the head of any lane exists HV
        self.allowedPassTime = 0
        self.schedulePower=schedulePower
        self.method=method 

    # vehicle's distance to IM
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
        passtime,_,_ = self.method(arrivalTime,HV,[(0,1),(2,3)],self.G1,self.G2,self.schedulePower)
        
        if passtime is not None:
            passingOrder = {}
            for l,lane in enumerate(self.vInRange):
                vlist = self.vInRange[lane]
                for i,v in enumerate(vlist):
                    if i >= self.schedulePower:
                        break
                    passingOrder[v] = sum([1 if t <  passtime[l][i] else 0 for lane in passtime for t in lane])
            self.passingOrder = passingOrder

    def stopBeforePass(self): #vehicles stop before entering the intersection, duration is equal to G         
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
                    self.allowedPassTime = max(self.allowedPassTime,currentTime + (self.G2 if self.existsHV else self.G1))
                    break
        
        for v in self.stoppedVehicle:
            if self.vDistance(v) >= self.stopRadius:
                self.stoppedVehicle.remove(v)


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
    def getAvgWaitTime(self):
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
        totalPTime = 0
        for v in IM.passTimes:
            totalPTime += IM.passTimes[v] - IM.enterTimes[v]
        return 0 if len(IM.passTimes) == 0 else totalPTime/len(IM.passTimes)


def run(schedulePower,schedule=True,method=MILPBased):
    """execute the TraCI control loop"""
    step = 0
    sensorRange = 500  #
    scheduleInterval = 1 # IM schedule interval(second)
    lastScheduleTime = -math.inf
    incomingLanes = ["1i","2i","3i","4i"]
    outboundLanes = ["2o","1o","4o","3o"]
    IM = IntersectionManager(incomingLanes,outboundLanes,sensorRange,scheduleInterval,schedulePower,method) 
    analysis = Analysis(outboundLanes)

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        IM.updateVInRange()
        IM.updateExceptedArrivalTime()
        if schedule and step * STEPLENGTH - lastScheduleTime >= scheduleInterval:
            lastScheduleTime = step * STEPLENGTH
            IM.schedule()
        IM.stopBeforePass()
        IM.startPass(step * STEPLENGTH,schedule=schedule)

        analysis.updateWaitTime()
        step += 1
        sys.stdout.flush()

    traci.close()
    avgWaitTime,cavWaitTime,hvWaitTime = analysis.getAvgWaitTime()
    return avgWaitTime,cavWaitTime,hvWaitTime


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


if __name__ == "__main__":
    RESULTPATH = "./results/SUMO_nosignaltest.csv"
    STEPLENGTH = 1 #time for a step
    G1 = 1
    G2 = 3
    MEANINTERVAL = 20 # average interval of incoming vehicle from each lane 
    TESTCOUNT    = 2 
    TIMESTEPS = 50 # number of time steps
    
    options = get_options()
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    #sumoBinary = "/mnt/c/Program Files (x86)/Eclipse/Sumo/bin/sumo-gui.exe"
    sumoBinary = "/mnt/c/Program Files (x86)/Eclipse/Sumo/bin/sumo.exe"
    scheduleWait = []
    unscheduleWait =  []
    schedulePowers = [12] # How much vehicles can the IM schedule
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
            avgTotal_unscheduled = 0
            avgCAV_unscheduled = 0
            avgHV_unscheduled = 0
            for test in range(TESTCOUNT	):
                generate_routefile(HVratio,MEANINTERVAL)
                traci.start([sumoBinary, "-c", "./sumocfg/cross.sumocfg","--tripinfo-output", "tripinfo.xml"])
                Wait,cavWait,hvWait = run(schedulePower,schedule = True)
                avgTotal += Wait
                avgCAV +=  cavWait
                avgHV += hvWait
                
                traci.start([sumoBinary, "-c", "./sumocfg/cross.sumocfg","--tripinfo-output", "tripinfo.xml"])
                Wait,cavWait,hvWait = run(schedulePower,schedule = False)
                avgTotal_unscheduled += Wait
                avgCAV_unscheduled +=  cavWait
                avgHV_unscheduled += hvWait

            scheduleTotal.append(avgTotal/TESTCOUNT	)
            scheduleCAV.append(avgCAV/TESTCOUNT	)
            scheduleHV.append(avgHV/TESTCOUNT	)

            unScheduleTotal.append(avgTotal_unscheduled/TESTCOUNT	)
            unScheduleCAV.append(avgCAV_unscheduled/TESTCOUNT	)
            unScheduleHV.append(avgHV_unscheduled/TESTCOUNT	)

    df = pd.DataFrame({
            'Scheduled': scheduleTotal,
            'Unscheduled': unScheduleTotal,
            'ScheduledCAV': scheduleCAV,
            'ScheduledHV': scheduleHV, 
            'UnscheduledCAV': unScheduleCAV,
            'UnscheduledHV': unScheduleHV,
        },index = np.arange(0,1.1,0.1))
    df.index.name = "HV ratio"
    df.to_csv(RESULTPATH)

