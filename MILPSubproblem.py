from copass import *

def solveDC(arrivalTime,HV,Copass,P1=1,P2=3):
	SUBCASESIZE = 4
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
	testCount = 10
	MILPcosts = []
	FCFScosts = []
	Ratios = []

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
		MILPcosts.append(round(MILPcost,2))
		FCFScosts.append(round(FCFScost,2))
		Ratios.append(round(MILPcost/FCFScost,2))
			
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

	plt.plot(np.arange(0.0,1.1,0.1),Ratios,label = "N=4")

	MILPcosts = []
	FCFScosts = []
	avgRuntimes = []
	Ratios = []
	for HVratio in np.arange(0,1.1,0.1):
		print("ratio:", HVratio)
		FCFScost = 0
		MILPcost = 0
		avgRuntime = 0
		successCount = testCount
		i = 0
		while i < testCount:
			arrivalTime,HV,Copass = GenerateTestCase(HVratio,meanInterval,4,10,[(0,1),(2,3)])

			t,cost,runtime = solveDC(arrivalTime,HV,Copass)
			if cost == -1: #gurobi solve fail
				continue
			avgRuntime += runtime
			MILPcost += cost 
			t,cost = solveFCFS(arrivalTime,HV,Copass)
			FCFScost += cost
			np.set_printoptions(precision=2)
			print('FCFS cost:',cost)
			print(t)
			i += 1

		avgRuntime /= testCount
		MILPcost /= testCount
		FCFScost /= testCount
		print(MILPcost)
		print(FCFScost)
		print(MILPcost/FCFScost)
		avgRuntimes.append(round(avgRuntime,3))
		MILPcosts.append(round(MILPcost,2))
		FCFScosts.append(round(FCFScost,2))
		Ratios.append(round(MILPcost/FCFScost,2))

	print(avgRuntimes,sum(avgRuntimes)/len(avgRuntimes))
	plt.plot(np.arange(0.0,1.1,0.1),Ratios,label = "N=10")


	plt.xlabel('HVRatio')
	plt.ylabel('MILP/FCFS')
	plt.legend()
	fig = plt.gcf() 
	fig.savefig('MILPFCFSRatio'+ str(meanInterval) + '.svg')