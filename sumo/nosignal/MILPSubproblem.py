from copass import *
import pandas as pd
import pickle

def solveDC(arrivalTime,HV,Copass,P1=1,P2=3,SUBCASESIZE=12):
	head = np.zeros(len(arrivalTime),dtype = 'int')
	N = []
	t = []
	for lane in arrivalTime:
		N.append(len(lane))
		t.append([])


	N = np.asarray(N)
	#print(N,head)
	currentTime = -P2
	runtime = 0
	
	while not np.array_equal(head,N):
		n = 0
		subcase = [0] * len(arrivalTime)
		while  n < SUBCASESIZE and not np.array_equal(head+subcase,N):
			headVs = [lane[head[i]+subcase[i]] if head[i]+subcase[i] < len(lane) else math.inf for i,lane in enumerate(arrivalTime)]
			firstv = min(headVs)
			#print(headVs)
			for i,a in enumerate(headVs):
			 	if a == firstv:
			 		subcase[i] += 1
			 		n += 1
			 		break 


		currentTime += P2
		subCaseA = []
		subCaseHV = []
		for i,a in enumerate(arrivalTime):
			subCaseA.append(arrivalTime[i][head[i]:head[i] + subcase[i]])
			subCaseHV.append(HV[i][head[i]:head[i] + subcase[i]])
			head[i] = head[i] + subcase[i] if head[i] + subcase[i] <= N[i] else N[i]

		#print(subCaseA)
		#print(subCaseHV)
		subt,cost,caseRuntime = solveMILP(subCaseA,subCaseHV,Copass,P1=P1,P2=P2,TSTART=currentTime,obj="throughput")
		if cost == -1:
			return  None,-1,-1
		runtime += caseRuntime
		currentTime = max(max(lane) if len(lane) > 0 else 0  for lane in subt)
		for i,lanet in enumerate(subt):
			t[i].extend(lanet) 

	#print("MILSUB_t: ", t)
	#print(currentTime)
	return t, currentTime , runtime


if __name__ == '__main__':
	meanInterval = 2
	
	testCount = 10
	MILPcosts = []
	FCFScosts = []
	Ratios1 = []

	for HVratio in np.arange(0,1.1,0.1):
		print("ratio:", HVratio)
		FCFScost = 0
		MILPcost = 0
		successCount = testCount
		i = 0
		while i < testCount:
			arrivalTime,HV,Copass = GenerateTestCase(HVratio,meanInterval,4,3,[(0,1),(2,3)])
			t,cost,runtime = solveMILP(arrivalTime,HV,Copass,obj='throughput')
			if cost == -1: #gurobi solve fail
				continue
			print("arrival:",arrivalTime)
			print('MILPcost:',cost,t)
			MILPcost += cost 
			t,cost = solveFCFS(arrivalTime,HV,Copass)
			print('FCFScost:',cost,t)
			FCFScost += cost
			np.set_printoptions(precision=2)
			#print(t)

			i += 1

		MILPcost /= testCount
		FCFScost /= testCount
		print('MILPcost:',MILPcost)
		print('FCFScost:',FCFScost)
		MILPcosts.append(round(MILPcost,3))
		FCFScosts.append(round(FCFScost,3))
		Ratios1.append(round(MILPcost/FCFScost,3))
			
	print(MILPcosts)
	print(FCFScosts)


	#plt.plot(np.arange(0.0,1.1,0.1),MILPcosts,label = "MILP")
	#plt.plot(np.arange(0.0,1.1,0.1),FCFScosts,label = "FCFS")
	#plt.xlabel('HVRatio')
	#plt.ylabel('Cost(s)')
	#plt.legend()
	#fig = plt.gcf() 
	#fig.savefig('MILPSUBvsFCFS'+ str(meanInterval) + '.svg')
	#exit()

	plt.plot(np.arange(0.0,1.1,0.1),Ratios1,label = "N=4")

	subcasesizes = [12]

	testCount = 10	
	MILPsubcosts = [[] for i in range(len(subcasesizes))]
	MILPcosts = []
	FCFScosts = []
	avgRuntimes = [[] for i in range(len(subcasesizes))]
	Ratios2 = [[] for i in range(len(subcasesizes))]
	for HVratio in np.arange(0,1.1,0.1):
		print("ratio:", HVratio)
		FCFScost = 0
		MILPcost = 0
		MILPsubcost = [0] * len(subcasesizes)
		avgRuntime = [0] * len(subcasesizes)
		
		successCount = testCount
		n = 0
		while n < testCount:
			arrivalTime,HV,Copass = GenerateTestCase(HVratio,meanInterval,4,9,[(0,1),(2,3)])
			milpt,milpcost,runtime = solveMILP(arrivalTime,HV,Copass,obj="throughput")
			t,fcfscost = solveFCFS(arrivalTime,HV,Copass)
			if milpcost == -1: #gurobi solve fail
				continue

			solveFail = False
			t = [0] * len(subcasesizes)
			cost = [0] * len(subcasesizes)
			runtime = [0] * len(subcasesizes)
			for i,subcasesize in enumerate(subcasesizes):
				t[i],cost[i],runtime[i] = solveDC(arrivalTime,HV,Copass,SUBCASESIZE=subcasesize)
				if cost[i] == -1: #gurobi solve fail
					solveFail = True
					break
				print("MILPsub cost:",cost[i])
			if solveFail:
				continue
			with open("./testcase/MILPSubproblem_" + str(i),"wb") as f:
				pickle.dump(arrivalTime,f)
				pickle.dump(HV,f)


			for i,subcasesize in enumerate(subcasesizes):
				avgRuntime[i] += runtime[i]
				MILPsubcost[i] += cost[i] 			
			FCFScost += fcfscost
			MILPcost += milpcost
			print("milp_t:",milpt)
			print("msub_t:",t[0]) 	
			#np.set_printoptions(precision=3)
			#print('FCFS cost:',cost)
			#print(t)
			n += 1

		MILPcost /= testCount
		FCFScost /= testCount
		print("FCFS,MILP: ",FCFScost,MILPcost)

		FCFScosts.append(round(FCFScost,3))
		MILPcosts.append(round(MILPcost,3))
		for i,subcasesize in enumerate(subcasesizes):
			avgRuntime[i] /= testCount
			MILPsubcost[i] /= testCount
			print(MILPsubcost[i])
			print(MILPsubcost[i]/FCFScost)
			avgRuntimes[i].append(round(avgRuntime[i],3))
			MILPsubcosts[i].append(round(MILPsubcost[i],3))

			Ratios2[i].append(round(MILPsubcost[i]/FCFScost,3))
			print('runtime:',avgRuntimes[i],sum(avgRuntimes[i])/len(avgRuntimes[i]))

	#plt.plot(np.arange(0.0,1.1,0.1),Ratios2,label = "N=10")
	#plt.xlabel('HVRatio')
	#plt.ylabel('MILP/FCFS')
	#plt.legend()
	#fig = plt.gcf() 
	#fig.savefig('MILPFCFSRatio'+ str(meanInterval) + '.svg')

	#print('runtime:', avgRuntimes)
	#df = pd.DataFrame({
    #    'MILP as Subproblem1': MILPsubcosts[0],
    #    'MILP as Subproblem2': MILPsubcosts[1],
        #'MILP as Subproblem3': MILPsubcosts[2],
    #    'MILP': MILPcosts,
    #    'FCFS': FCFScosts,
    #    },index = np.arange(0,1.1,0.1))
	#df.index.name = "HV ratio"
	#df.to_csv("MILPSUB_vsFCFS_vsMILP.csv")

	df = pd.DataFrame({
        'N=12 (No subcasing)': Ratios1,
        'N=36': Ratios2[0],
        },index = np.arange(0,1.1,0.1))
	df.index.name = "HV ratio"
	df.to_csv("MILPSUB_ratiostest.csv")