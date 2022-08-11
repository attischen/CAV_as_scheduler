import sys
import pandas as pd
import pickle

sys.path.append('../../')
from CAV_as_scheduler.Gurobi.MILP import *


def MILPBased(arrivalTime,HV,nonConflictingTrajectory,G1,G2,SUBCASESIZE):
	head = np.zeros(len(arrivalTime),dtype = 'int')
	N = []
	t = []
	for lane in arrivalTime:
		N.append(len(lane))
		t.append([])

	N = np.asarray(N)
	currentTime = -G2
	runtime = 0
	
	while not np.array_equal(head,N):
		n = 0
		subcase = [0] * len(arrivalTime)
		while  n < SUBCASESIZE and not np.array_equal(head+subcase,N):
			headVs = [lane[head[i]+subcase[i]] if head[i]+subcase[i] < len(lane) else math.inf for i,lane in enumerate(arrivalTime)]
			firstv = min(headVs)
			for i,a in enumerate(headVs):
			 	if a == firstv:
			 		subcase[i] += 1
			 		n += 1
			 		break 


		currentTime += G2
		subCaseA = []
		subCaseHV = []
		for i,a in enumerate(arrivalTime):
			subCaseA.append(arrivalTime[i][head[i]:head[i] + subcase[i]])
			subCaseHV.append(HV[i][head[i]:head[i] + subcase[i]])
			head[i] = head[i] + subcase[i] if head[i] + subcase[i] <= N[i] else N[i]

		subt,cost,caseRuntime = MILP(subCaseA,subCaseHV,nonConflictingTrajectory,G1=G1,G2=G2,TSTART=currentTime,obj="throughput")
		runtime += caseRuntime
		currentTime = max(max(lane) if len(lane) > 0 else 0  for lane in subt)
		for i,lanet in enumerate(subt):
			t[i].extend(lanet) 
	return t, currentTime , runtime


if __name__ == '__main__':
	MEANINTERVAL = 2
	TESTCOUNT = 1
	RESULTPATH_RATIO = "./results/MILPB_ratio.csv"
	RESULTPATH_MILPB_MILP_FCFS = "./results/MILPB_FCFS_MILP.csv"
	G1 = 1
	G2 = 3
	MILPcosts = []
	FCFScosts = []
	Ratios1 = []

	#The MILP(case size = 12) for our MILPB method(subcase size = 12) to compare improvement ratio to 
	for HVratio in np.arange(0,1.1,0.1):
		print("ratio:", HVratio)
		FCFScost = 0
		MILPcost = 0
		successCount = TESTCOUNT	
		i = 0
		while i < TESTCOUNT	:
			arrivalTime,HV,nonConflictingTrajectory = GenerateTestCase(HVratio,MEANINTERVAL,4,3,[(0,1),(2,3)])
			t,cost,runtime = MILP(arrivalTime,HV,nonConflictingTrajectory,obj='throughput')
			MILPcost += cost 
			t,cost = FCFS(arrivalTime,HV,nonConflictingTrajectory)
			FCFScost += cost

			i += 1

			MILPcost /= TESTCOUNT	
		FCFScost /= TESTCOUNT	
		MILPcosts.append(round(MILPcost,3))
		FCFScosts.append(round(FCFScost,3))
		Ratios1.append(round(MILPcost/FCFScost,3))
	

	subcasesizes = [4,12,20]

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
		
		successCount = TESTCOUNT	
		n = 0
		while n < TESTCOUNT	:
			arrivalTime,HV,nonConflictingTrajectory = GenerateTestCase(HVratio,MEANINTERVAL,4,9,[(0,1),(2,3)])
			milpt,milpcost,runtime = MILP(arrivalTime,HV,nonConflictingTrajectory,obj="throughput")
			t,fcfscost = FCFS(arrivalTime,HV,nonConflictingTrajectory)
			if milpcost == -1: #gurobi solve fail
				continue

			solveFail = False
			t = [0] * len(subcasesizes)
			cost = [0] * len(subcasesizes)
			runtime = [0] * len(subcasesizes)
			for i,subcasesize in enumerate(subcasesizes):
				t[i],cost[i],runtime[i] = MILPBased(arrivalTime,HV,nonConflictingTrajectory,G1,G2,SUBCASESIZE=subcasesize)

			with open("./testcase/MILPSubproblem_" + str(i),"wb") as f:
				pickle.dump(arrivalTime,f)
				pickle.dump(HV,f)


			for i,subcasesize in enumerate(subcasesizes):
				avgRuntime[i] += runtime[i]
				MILPsubcost[i] += cost[i] 			
			FCFScost += fcfscost
			MILPcost += milpcost 	
			n += 1

		MILPcost /= TESTCOUNT	
		FCFScost /= TESTCOUNT	
		print("FCFS,MILP: ",FCFScost,MILPcost)

		FCFScosts.append(round(FCFScost,3))
		MILPcosts.append(round(MILPcost,3))
		for i,subcasesize in enumerate(subcasesizes):
			avgRuntime[i] /= TESTCOUNT
			MILPsubcost[i] /= TESTCOUNT
			avgRuntimes[i].append(round(avgRuntime[i],3))
			MILPsubcosts[i].append(round(MILPsubcost[i],3))
			Ratios2[i].append(round(MILPsubcost[i]/FCFScost,3))

	print('runtime:', avgRuntimes)
	df = pd.DataFrame({
        'MILPB1': MILPsubcosts[0],
        'MILPB2': MILPsubcosts[1],
        'MILPB3': MILPsubcosts[2],
        'MILP': MILPcosts,
        'FCFS': FCFScosts,
        },index = np.arange(0,1.1,0.1))
	df.index.name = "HV ratio"
	df.to_csv(RESULTPATH_MILPB_MILP_FCFS)

	df = pd.DataFrame({
        'N=12 (No subcasing)': Ratios1,
        'N=36': Ratios2[0],
        },index = np.arange(0,1.1,0.1))
	df.index.name = "HV ratio"
	df.to_csv(RESULTPATH_RATIO)