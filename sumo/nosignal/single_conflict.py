import gurobipy as gp
from gurobipy import *
import numpy as np

def solve(arrivalTime,HV,P1=1,P2=3):
    def printSolution():
        fail = 0
        IntBinVars = [H,Ord,Next,y,r,s,n]
        for Vars in IntBinVars:
            for v in Vars.flatten():
                if not v.X.is_integer():
                    print("Solution not Int:",v)
                    fail = 1
                    break
        if fail:
            return 

        if m.status == GRB.OPTIMAL:
            print('\nCost: %g' % m.objVal)
            print(t)
            #for Vars in IntBinVars:
            #    print(Vars)
            
        else:       
            print('No optimal solution')

    # Model
    m = gp.Model("schedule")

    # These params have to be set well to not allow near solutions (e.g. 0.0000001 BIN to bypass big-M constraints) 
    m.setParam("IntegralityFocus",1) 
    m.setParam("IntFeasTol",1e-5) #1e-9 best , but sacrifice performance
    M = 1e4    
    M1 = 1e3

    Nlane = len(arrivalTime) #number of lanes
    t = [] # pass time of v[l][i]
    N = []         # number of vehicles in lane l
    hv = []        # is v[l][i] HV?
    H = []         # is v[l'][j] the head vehicle when v[l][i] pass?
    Ord = []       # Does v[l][i] pass before v[l'][j] ?
    n = []         # the order of passing of v[l][i]  
    Next = []      # v[l][i]'s next passing vehicle is v[l'][j]? 
    y,r,s = [],[],[] # auxilary variables

    a = np.asarray(arrivalTime,dtype = 'object')
    hv = np.asarray(HV,dtype = 'object')
    N = []


    for arrival in arrivalTime:
        N.append(len(arrival))

    for L,arrivalL in enumerate(arrivalTime):
        t.append([])
        n.append([])
        y.append([])
        r.append([])
        s.append([])
        H.append([])
        Ord.append([])
        Next.append([])
        for i in range(N[L]):
            t[L].append(m.addVar(vtype=GRB.CONTINUOUS, name="t"+str(L)+ '_' + str(i) ))
            n[L].append(m.addVar(vtype=GRB.INTEGER, name="n"+str(L)+ '_' + str(i) ))
            y[L].append(m.addVar(vtype=GRB.BINARY, name="y"+str(L)+ '_' + str(i) ))
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
                    Next[L][i][L2].append(m.addVar(vtype=GRB.BINARY, name="Next"+str(L)+ '_' + str(i)  + '_'  +  str(L2) +'_'+str(j) ))
                    r[L][i][L2].append(m.addVar(vtype=GRB.BINARY, name="r"+ str(L)+ '_' + str(i)  + '_'  +  str(L2) +'_'+str(j) ))
                    s[L][i][L2].append(m.addVar(vtype=GRB.BINARY, name="s"+ str(L)+ '_' + str(i)  + '_'  +  str(L2) +'_'+str(j) ))
                H[L][i][L2].append(m.addVar(vtype=GRB.BINARY, name="H"+str(L)+ '_' + str(i)  + '_'  +  str(L2) +'_'+str(N[L2])))

    t = np.asarray(t)
    H = np.asarray(H)
    Ord = np.asarray(Ord)
    n = np.asarray(n)
    Next = np.asarray(Next)
    y = np.asarray(y)
    r = np.asarray(r)
    s = np.asarray(s)
    
    mint = m.addVar(vtype=GRB.INTEGER,name = "mint")
    lastlist = []
    for  l in range(Nlane):
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
                m.addConstr( t[l][i] >= a[l][i] ,"c1")
        
        
        #pass time
        
        for l in range(Nlane):
            for i in range(N[l]):
                m.addConstr( gp.quicksum( t[l2][j] * Next[l][i][l2][j]  for l2 in range(Nlane) for j in range(N[l2]) ) - t[l][i] >= P1 + M1*( gp.quicksum( Next[l][i][l2][j] for l2 in range(Nlane) for j in range(N[l2])) - 1) ,"c2_1")
                m.addConstr( gp.quicksum( -hv[l2][j] * H[l][i][l2][j]  for l2 in range(Nlane) for j in range(N[l2])) >= -M*y[l][i],"c2_2" )
                m.addConstr( gp.quicksum( t[l2][j] * Next[l][i][l2][j]  for l2 in range(Nlane) for j in range(N[l2]) ) - t[l][i] >= P2 + M1*( gp.quicksum(Next[l][i][l2][j]  for l2 in range(Nlane) for j in range(N[l2])) - 1) + M1*(y[l][i] - 1)  ,"c2_3" )
        
        


        for l in range(Nlane):
            for i in range(N[l]):
                for l2 in [l2 for l2 in range(Nlane) if l2 != l]:
                    for j in range(N[l2]):
                        m.addConstr( M*H[l][i][l2][j]*HV[l2][j] + (a[l][i] - a[l2][j]) <= M ,"c3")
            
        return

    def AddOtherConstraints():
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
    printSolution()

arrivalTime = [[0,1,2],[0,1,6],[2,4,7]]
HV = [[0,0,1],[0,1,1],[1,0,0]]

solve(arrivalTime,HV)
   