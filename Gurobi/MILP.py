import gurobipy as gp
from gurobipy import *
import numpy as np
import random
import matplotlib.pyplot as plt
import pandas as pd
from collections.abc import Iterable
RESULTPATH_MILP_FCFS = "./results/MILP_FCFS.csv"
RESULTPATH_WAITTIME = "./results/MILP_waittime.csv"
def flatten(l):
    for el in l:
        if isinstance(el, Iterable) and not isinstance(el, (str, bytes)):
            yield from flatten(el)
        else:
            yield el

def GenerateTestCase(HVratio,arrivalInterval,laneCount,N,Copass):
    arrivalTime = []
    HV = []
    for l in range(laneCount):
        arrivalTime.append([])
        HV.append([])
        t = 5
        for n in range(N):
            #t += random.random() * arrivalInterval*2
            t += (arrivalInterval) * -math.log(1- random.random()) 
            arrivalTime[l].append(t)
            HV[l].append(1 if random.random() < HVratio else 0)
    return arrivalTime,HV,Copass

def MILP(arrivalTime,HV,Copass,P1=1,P2=3,TSTART=0,HVSchedulable=False,obj="throughput"):
    def printSolution():
        fail = 0
        IntBinVars = [H,Ord,y,r,s,n,q]

        for Vars in IntBinVars:
            #print(flatten(Vars))
            for var in flatten(Vars):
                #print(var)
                if not var.X.is_integer():
                    print("Solution not Int:",var)
                    fail = 1
                    return None,-1
                    
        if fail:
            return None,-1

        if m.status == GRB.OPTIMAL:
            #print('\nCost: %g' % m.objVal)
            tvars = []
            for lanet in t:
                tvars.append([round(tvar.X,2) for tvar in lanet])
            #for Vars in IntBinVars:
            #    print(Vars)
            return tvars,m.objVal
        else:       
            print('No optimal solution')
            return None,-1

    # Model
    m = gp.Model("schedule")
    m.setParam('OutputFlag', 0) #mute output 
    # These params have to be set well to not allow near solutions (e.g. 0.0000001 BIN to bypass big-M constraints) 
    m.setParam("IntegralityFocus",1) 
    m.setParam("IntFeasTol",1e-5) #1e-9 best , but sacrifice performance
    M = 1e3    
    M1 = 1e5

    Nlane = len(arrivalTime) #number of lanes
    t = [] # pass time of v[l][i]
    N = []         # number of vehicles in lane l
    hv = []        # is v[l][i] HV?
    H = []         # is v[l'][j] the head vehicle when v[l][i] pass?
    Ord = []       # Does v[l][i] pass before v[l'][j] ?
    n = []         # the order of passing of v[l][i]  
    Next = []      # v[l][i]'s next passing vehicle is v[l'][j]? 
    y,q,r,s = [],[],[],[] # auxilary variables

    a = np.asarray(arrivalTime,dtype = 'object')
    hv = np.asarray(HV,dtype = 'object')
    N = []


    for arrival in arrivalTime:
        N.append(len(arrival))

    for L,arrivalL in enumerate(arrivalTime):
        t.append([])
        n.append([])
        y.append([])
        q.append([])
        r.append([])
        s.append([])
        H.append([])
        Ord.append([])
        Next.append([])
        for i in range(N[L]):
            t[L].append(m.addVar(vtype=GRB.CONTINUOUS, name="t"+str(L)+ '_' + str(i) ))
            n[L].append(m.addVar(vtype=GRB.INTEGER, name="n"+str(L)+ '_' + str(i) ))
            y[L].append(m.addVar(vtype=GRB.BINARY, name="y"+str(L)+ '_' + str(i) ))
            q[L].append(m.addVar(vtype=GRB.BINARY, name="q"+str(L)+ '_' + str(i) ))
            r[L].append([])
            s[L].append([])
            H[L].append([])
            Ord[L].append([])
            Next[L].append([])
            for L2 in range(Nlane):
                H[L][i].append([])
                Ord[L][i].append([])
                Next[L][i].append([])
                r[L][i].append([])
                s[L][i].append([])
                for j in range(N[L2]):
                    H[L][i][L2].append(m.addVar(vtype=GRB.BINARY, name="H"+str(L)+ '_' + str(i)  + '_'  +  str(L2) +'_'+str(j) ))
                    Ord[L][i][L2].append(m.addVar(vtype=GRB.BINARY, name="Ord"+str(L)+ '_' + str(i)  + '_'  +  str(L2) +'_'+str(j) ))
                    #Next[L][i][L2].append(m.addVar(vtype=GRB.BINARY, name="Next"+str(L)+ '_' + str(i)  + '_'  +  str(L2) +'_'+str(j) ))
                    #r[L][i][L2].append(m.addVar(vtype=GRB.BINARY, name="r"+ str(L)+ '_' + str(i)  + '_'  +  str(L2) +'_'+str(j) ))
                    #s[L][i][L2].append(m.addVar(vtype=GRB.BINARY, name="s"+ str(L)+ '_' + str(i)  + '_'  +  str(L2) +'_'+str(j) ))
                H[L][i][L2].append(m.addVar(vtype=GRB.BINARY, name="H"+str(L)+ '_' + str(i)  + '_'  +  str(L2) +'_'+str(N[L2])))

    t = np.asarray(t)
    H = np.asarray(H)
    Ord = np.asarray(Ord)
    n = np.asarray(n)
    Next = np.asarray(Next)
    y = np.asarray(y)
    q = np.asarray(q)
    r = np.asarray(r)
    s = np.asarray(s)
    
    

    if obj == "waitTime":
        m.setObjective(gp.quicksum(t[l][i] for l in range(Nlane) for i in range(N[l])), GRB.MINIMIZE)
    else:
        mint = m.addVar(vtype=GRB.CONTINUOUS,name = "mint")
        lastlist = []
        for  l in range(Nlane):
            if len(t[l]) > 0:
                lastlist.append(t[l][-1])
        m.addConstr(mint == max_(lastlist))
        m.setObjective(mint, GRB.MINIMIZE)
    

    #Main Constraints: 
    def AddMainConstraints():
        
        for l in range(Nlane):
            for i in range(N[l]):
                m.addConstr( Ord[l][i][l][i] == 0 ,"c0_1")
                for j in range(i + 1,N[l]):
                    m.addConstr( Ord[l][i][l][j] == 1 ,"c0_2")
        


        for l in range(Nlane):
            for i in range(N[l]):
                m.addConstr( t[l][i] >= a[l][i] ,"c1_1")
                m.addConstr( t[l][i] >= TSTART ,"c1_2")
        
        
        #pass time
        for l in range(Nlane):
            for i in range(N[l]):
                otherlanes = [x for x in range(Nlane) if x != l]
                #if ( i ==  N[l] - 1): # possibile last vehicle
                #    m.addConstr( gp.quicksum( H[l][i][l2][j] for l2 in otherlanes for j in range(N[l2]) ) + M*(q[l][i]-1) <= 0,"c2_1_1")
                #else:
                m.addConstr( q[l][i] == 0,"c2_1_2")

                m.addConstr( gp.quicksum( -hv[l2][j] * H[l][i][l2][j]  for l2 in range(Nlane) for j in range(N[l2])) >= -M*y[l][i],"c2_2" )
                for l2 in range(Nlane):
                    for j in range(N[l2]):
                        if (l,l2) in Copass or (l2,l) in Copass:
                            m.addConstr(  t[l2][j]  - t[l][i] + M*(1-Ord[l][i][l2][j]) + M*q[l][i] >= 0 ,"c2_3")
                        else:
                            m.addConstr(  t[l2][j]  - t[l][i] + M*(1-Ord[l][i][l2][j]) + M*q[l][i] >= P1 ,"c2_4")
                            m.addConstr(  t[l2][j]  - t[l][i] + M*(1-Ord[l][i][l2][j]) + M*q[l][i] >= P2 + M *(y[l][i]-1),"c2_5")
                                
        if HVSchedulable:
            return  

        for l in range(Nlane):
            for i in range(N[l]):
                for l2 in [l2 for l2 in range(Nlane) if l2 != l and not((l,l2) in Copass or (l2,l) in Copass)]:
                    for j in range(N[l2]):
                        m.addConstr( M*H[l][i][l2][j]*HV[l2][j] + (a[l][i] - a[l2][j]) <= M ,"c3")
            
        

    def AddOtherConstraints():
        #m.addConstr( gp.quicksum(q[l][i] for l in range(Nlane) for i in range(N[l])) == 1, "c_q")
    #Other Constraints:
        for l in range(Nlane):
            for i in range(N[l]):
                for l2 in range(Nlane):
                    for j in range(N[l2]):
                        if (l,i) == (l2,j):
                            continue
                        m.addConstr( Ord[l][i][l2][j] + Ord[l2][j][l][i] == 1 ,"c4_1")

                        for l3 in range(Nlane):
                            for k in range(N[l3]):
                                if (l,i) == (l3,k) or (l2,j) == (l3,k):
                                    continue 
                                m.addConstr( Ord[l][i][l2][j] + Ord[l2][j][l3][k] - 1 <= Ord[l][i][l3][k]  ,"c4_2")

        for l in range(Nlane):
            for i in range(N[l]):
                    m.addConstr( gp.quicksum( Ord[l2][j][l][i] for l2 in range(Nlane) for j in range(N[l2]) ) == n[l][i] ,"c5")


        for l in range(Nlane):
                for i in range(N[l]):
                    m.addConstr( H[l][i][l][i] == 1 ,"c6_1")
                    for l2 in [l2 for l2 in range(Nlane) if l2 != l]:
                        m.addConstr( gp.quicksum( H[l][i][l2][j]  for j in range(N[l2] + 1) ) == 1 ,"c6_2")
                        m.addConstr( gp.quicksum( Ord[l2][j][l][i]  for j in range(N[l2]) )  == gp.quicksum( j*H[l][i][l2][j]  for j in range(N[l2] + 1) ) ,"c6_3")

        return 

        for l in range(Nlane):
            for i in range(N[l]):
                for l2 in range(Nlane):
                    for j in range(N[l2]):
                        m.addConstr( r[l][i][l2][j] + Next[l][i][l2][j] + s[l][i][l2][j] == 1 ,"c7_1")
                        m.addConstr( Next[l][i][l2][j]*(n[l][i] + 1) + s[l][i][l2][j] * (n[l][i] + 2) <= n[l2][j] ,"c7_2")
                        m.addConstr( Next[l][i][l2][j]*(n[l][i] + 1) + s[l][i][l2][j] * M + r[l][i][l2][j]* n[l][i] >= n[l2][j] ,"c7_3")
    
    AddOtherConstraints()
    AddMainConstraints()

    m.update()
    m.optimize()
    tvars,cost = printSolution()
    runtime = m.Runtime
    return tvars,cost,runtime

def FCFS(arrivalTime,HV,Copass,P1=1,P2=3):
    Nlane = len(arrivalTime)
    t = [] # time of entering citical zone
    passTime = [] # time required to exit critical zone,P1 or P2
    N = []
    for i,lane in enumerate(arrivalTime):
        N.append(len(lane))
        t.append([])
        passTime.append([])
        for j in range(len(lane)):
            t[i].append(0)
            passTime[i].append(0)
    
    a = np.asarray(arrivalTime,dtype = 'object')
    hv = np.asarray(HV,dtype = 'object')
    t = np.asarray(t,dtype='float')
    passTime = np.asarray(passTime)
    N = np.asarray(N)

    head = np.zeros([Nlane],dtype = 'int')

    prev = None
    prevT = -100

    P2Until = -100
    passing = []
    while not np.array_equal(head,N):
        #print(prev,prevT)
        Candidate = []
        bestT = 1e5
        best = (-1,-1)
        for lane in range(Nlane):
            if head[lane] == N[lane]:
                continue
            Candidate.append((lane,head[lane]))  
        
        # fastest colane
        for c in Candidate:
            minT = a[c[0]][c[1]]
            for p in passing:
                if not ((p[0],c[0]) in Copass or  (c[0],p[0]) in Copass) and t[p[0]][p[1]] + passTime[p[0]][p[1]] > minT:
                    minT = t[p[0]][p[1]] + passTime[p[0]][p[1]]

            if best == (-1,-1) or a[c[0]][c[1]] < a[best[0]][best[1]]:
                best = c
                bestT = minT
            
        passTime[best[0]][best[1]] = P1 
        if P2Until >= bestT:
            passTime[best[0]][best[1]] = P2
        for l,i in enumerate(head):
            if i == N[l]:
                continue
            if hv[l][i] == 1:
                passTime[best[0]][best[1]] = P2

        if hv[best[0]][best[1]] == 1:
            P2Until = bestT + passTime[best[0]][best[1]]

        head[best[0]] += 1
        t[best[0]][best[1]] = bestT
        prev = best
        prevT = bestT
        passing.append(best)
    


    return t,t.max() 

    #for v in arrivalTime[]

def checkWaitTime(arrivalTime,passTime,HV):
    CAVCount = 0
    HVCount = 0
    avgHVWaitTime = 0
    avgCAVWaitTime = 0
    for aT,pT,hv in zip(arrivalTime,passTime,HV):
        for a,p,h in zip(aT,pT,hv):
            if h:
                avgHVWaitTime += p - a 
                HVCount += 1 
            else:
                avgCAVWaitTime += p - a
                CAVCount += 1


    avgCAVWaitTime = -1 if CAVCount == 0 else avgCAVWaitTime / CAVCount
    avgHVWaitTime = -1 if HVCount == 0 else  avgHVWaitTime / HVCount
    print(avgHVWaitTime,avgCAVWaitTime)
    return avgHVWaitTime,avgCAVWaitTime

if __name__ == "__main__":
    meanInterval = 2
    testCount = 1
    MILPcosts = []
    MILP2costs = []

    FCFScosts = []
    avgRuntimes = []
    Ratios = []
    HVWaitTimes = []
    CAVWaitTimes = []
    HVWaitTimes2 = []
    CAVWaitTimes2 = []
    for HVratio in np.arange(0.0,1.1,0.1):
        print("ratio:", HVratio)
        FCFScost = 0
        MILPcost = 0
        MILP2cost = 0
        avgRuntime = 0
        avgHVWaitTime = 0
        avgCAVWaitTime = 0
        avgHVWaitTime2 = 0
        avgCAVWaitTime2 = 0
        successCount = testCount
        i = 0
        while i < testCount:
            arrivalTime,HV,Copass = GenerateTestCase(HVratio,meanInterval,4,5,[(0,1),(2,3)])
            #print(arrivalTime)
            #print(HV)
            #print(Copass)
 
            t,cost,runtime = MILP(arrivalTime,HV,Copass,HVSchedulable = False)
            if cost == -1: #gurobi solve fail
                continue
            HVWaitTime,CAVWaitTime = checkWaitTime(arrivalTime,t,HV)
            avgHVWaitTime += HVWaitTime
            avgCAVWaitTime += CAVWaitTime
            avgRuntime += runtime
            
            
            t,cost2,runtime = MILP(arrivalTime,HV,Copass,HVSchedulable = True)
            if cost2 == -1: #gurobi solve fail
                continue

            MILPcost += cost 
            MILP2cost += cost2 

            t,cost = FCFS(arrivalTime,HV,Copass)
            HVWaitTime,CAVWaitTime = checkWaitTime(arrivalTime,t,HV)
            avgHVWaitTime2 += HVWaitTime
            avgCAVWaitTime2 += CAVWaitTime
            FCFScost += cost
            np.set_printoptions(precision=2)
            print('FCFS cost:',cost)
            print(t)
            i += 1

        avgHVWaitTime /= testCount
        avgCAVWaitTime /= testCount
        avgHVWaitTime2 /= testCount
        avgCAVWaitTime2 /= testCount
        avgRuntime /= testCount
        MILPcost /= testCount
        MILP2cost /= testCount
        FCFScost /= testCount
        HVWaitTimes.append(round(avgHVWaitTime,3))
        CAVWaitTimes.append(round(avgCAVWaitTime,3))
        HVWaitTimes2.append(round(avgHVWaitTime2,3))
        CAVWaitTimes2.append(round(avgCAVWaitTime2,3))
        avgRuntimes.append(round(avgRuntime,3))
        MILPcosts.append(round(MILPcost,3))
        MILP2costs.append(round(MILP2cost,3))
        FCFScosts.append(round(FCFScost,3))
        Ratios.append(round(MILPcost/FCFScost,3)) 

    df = pd.DataFrame({
        'MILP': MILPcosts,
        'MILP(HV schedulable)': MILP2costs,
        'FCFS': FCFScosts,
        'runtime' : avgRuntimes,
        },index = np.arange(0,1.1,0.1))
    df.index.name = "HV ratio"
    df.to_csv(RESULTPATH_MILP_FCFS)

    df = pd.DataFrame({
        'MILP-HV': HVWaitTimes,
        'MILP-CAV': CAVWaitTimes,
        'FCFS-HV': HVWaitTimes2,
        'FCFS-CAV': CAVWaitTimes2,
        },index = np.arange(0,1.1,0.1))
    df.index.name = "HV ratio"
    df.to_csv(RESULTPATH_WAITTIME)
