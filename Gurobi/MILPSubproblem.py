from copass import *
import pandas as pd

def solveDC(arrivalTime,HV,Copass,P1=1,P2=3,SUBCASESIZE=4):
	head = np.zeros(len(arrivalTime),dtype = 'int')
	N = []
	t = []
	for lane in arrivalTime:
		N.append(len(lane))
		t.append([])

	N = np.asarray(N)
	print(N,head)
	currentTime = -P2
	runtime = 0
	while not np.array_equal(head,N):
		currentTime += P2
		subCaseA = []
		subCaseHV = []
		for i,a in enumerate(arrivalTime):
			subCaseA.append(arrivalTime[i][head[i]:head[i] + SUBCASESIZE])
			subCaseHV.append(HV[i][head[i]:head[i] + SUBCASESIZE])
			head[i] = head[i] + SUBCASESIZE if head[i] + SUBCASESIZE <= N[i] else N[i]

		print(subCaseA)
		print(subCaseHV)
		subt,cost,caseRuntime = solveMILP(subCaseA,subCaseHV,Copass,P1,P2,TSTART=currentTime)
		if cost == -1:
			return  None,-1,-1
		runtime += caseRuntime
		currentTime = cost
		print(subt)
		for i,lanet in enumerate(subt):
			t[i].extend(lanet) 

	print(t)
	print(currentTime)
	return t, currentTime , runtime


if __name__ == '__main__':
	meanInterval = 2
	testCount = 30
	
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
			arrivalTime,HV,Copass = GenerateTestCase(HVratio,meanInterval,4,4,[(0,1),(2,3)])

			t,cost,runtime = solveMILP(arrivalTime,HV,Copass)
			if cost == -1: #gurobi solve fail
				continue

			MILPcost += cost 
			t,cost = solveFCFS(arrivalTime,HV,Copass)
			FCFScost += cost
			np.set_printoptions(precision=2)
			print('FCFS cost:',cost)
			print(t)
			i += 1

		MILPcost /= testCount
		FCFScost /= testCount
		print(MILPcost)
		print(FCFScost)
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

	subcasesizes = [1,2]
	MILPcosts = [[] for i in range(len(subcasesizes))]
	FCFScosts = []
	avgRuntimes = [[] for i in range(len(subcasesizes))]
	Ratios2 = [[] for i in range(len(subcasesizes))]
	for HVratio in np.arange(0,1.1,0.1):
		print("ratio:", HVratio)
		FCFScost = 0
		MILPcost = [0] * len(subcasesizes)
		avgRuntime = [0] * len(subcasesizes)
		
		successCount = testCount
		n = 0
		while n < testCount:
			arrivalTime,HV,Copass = GenerateTestCase(HVratio,meanInterval,4,10,[(0,1),(2,3)])

			solveFail = False

			t = [0] * len(subcasesizes)
			cost = [0] * len(subcasesizes)
			runtime = [0] * len(subcasesizes)
			for i,subcasesize in enumerate(subcasesizes):
				t[i],cost[i],runtime[i] = solveDC(arrivalTime,HV,Copass,SUBCASESIZE=subcasesize)
				if cost[i] == -1: #gurobi solve fail
					solveFail = True
					break
			if solveFail:
				continue

			for i,subcasesize in enumerate(subcasesizes):
				avgRuntime[i] += runtime[i]
				MILPcost[i] += cost[i] 

			t,cost = solveFCFS(arrivalTime,HV,Copass)
			FCFScost += cost
			np.set_printoptions(precision=3)
			print('FCFS cost:',cost)
			print(t)
			n += 1

		FCFScost /= testCount
		print(FCFScost)
		FCFScosts.append(round(FCFScost,3))
		for i,subcasesize in enumerate(subcasesizes):
			avgRuntime[i] /= testCount
			MILPcost[i] /= testCount
			print(MILPcost[i])
			print(MILPcost[i]/FCFScost)
			avgRuntimes[i].append(round(avgRuntime[i],3))
			MILPcosts[i].append(round(MILPcost[i],3))
			Ratios2[i].append(round(MILPcost[i]/FCFScost,3))
			print(avgRuntimes[i],sum(avgRuntimes[i])/len(avgRuntimes[i]))

	#plt.plot(np.arange(0.0,1.1,0.1),Ratios2,label = "N=10")
	#plt.xlabel('HVRatio')
	#plt.ylabel('MILP/FCFS')
	#plt.legend()
	#fig = plt.gcf() 
	#fig.savefig('MILPFCFSRatio'+ str(meanInterval) + '.svg')

	df = pd.DataFrame({
        'MILP as Subproblem1': MILPcosts[0],
        'MILP as Subproblem2': MILPcosts[1],
        #'MILP as Subproblem4': MILPcosts[2],
        'FCFS': FCFScosts,
        },index = np.arange(0,1.1,0.1))
	df.index.name = "HV ratio"
	df.to_csv("MILPSUB_vsFCFStest_12.csv")

	#df = pd.DataFrame({
    #    'N_i=4 (No subcasing)': Ratios1,
    #    'N_i=10': Ratios2,
    #    },index = np.arange(0,1.1,0.1))
	#df.index.name = "HV ratio"
	#df.to_csv("MILPSUB_ratios.csv")