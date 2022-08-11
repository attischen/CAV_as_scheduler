from MILP import *
import time 
import csv
import pandas as pd
RESULTPATH = "./results/DP.csv"

def DP(arrivalTime,HV,nonConflictingTrajectories,G1=1,G2=3):
	zeros =  np.zeros(len(arrivalTime),dtype = 'int')
	table = {} # DP table store the solved pass time of each vehicle
	def MinPass(Pass):
		if tuple(Pass) in table:
			return table[tuple(Pass)]

		if np.array_equal(Pass,zeros):
			table[tuple(Pass)] = 0
			return 0

		mint = math.inf
		for i,a in enumerate(arrivalTime):
			if Pass[i] == 0:
				continue

			Pass_ = np.array(Pass, copy=True)
			Pass_[i] -= 1  
			t = max(a[Pass[i] - 1],MinPass(Pass_) + PassTime(Pass_),Legal(Pass_,i))
			if t < mint:
				mint = t
		table[tuple(Pass)] = mint
		return mint

	# get the pass time(time gap) based on the current state of lanes
	def PassTime(Pass):
		for i,n in enumerate(Pass):
			if n >= len(HV[i]):
				continue
			if HV[i][n]:
				return G2
		return G1

	# if the vehicle from lane i can pass next based on the state of lanes (No passing before a HV which arrived earlier)
	def Legal(Pass,i):
		for j,n in enumerate(Pass):
			if j == i:
				continue
			if n >= len(HV[j]):
				continue
			if HV[j][n] and arrivalTime[j][n] < arrivalTime[i][Pass[i]]:
				return math.inf 
		return 0

	Pass = []
	for a in arrivalTime:
		Pass.append(len(a))

	Pass = np.asarray(Pass)
	startTime  = time.time()
	cost = MinPass(Pass)
	runtime = time.time() - startTime
	return cost,runtime

if __name__ == '__main__':
	MEANINTERVAL = 2
	TESTCOUNT = 1
	DPcosts = []
	FCFScosts = []
	avgRuntimes = []
	Ratios = []
	for HVratio in np.arange(0,1.1,0.1):
		FCFScost = 0
		DPcost = 0
		avgRuntime = 0
		successCount = TESTCOUNT
		i = 0
		while i < TESTCOUNT:
			arrivalTime,HV,nonConflictingTrajectories = GenerateTestCase(HVratio,MEANINTERVAL,4,10,[])

			cost,runtime = DP(arrivalTime,HV,nonConflictingTrajectories)
			DPcost += cost 
			t,cost = FCFS(arrivalTime,HV,nonConflictingTrajectories)
			FCFScost += cost
			avgRuntime += runtime
			np.set_printoptions(precision=3)
			i += 1
		avgRuntime /= TESTCOUNT
		DPcost /= TESTCOUNT
		FCFScost /= TESTCOUNT
		avgRuntimes.append(round(avgRuntime,3))
		DPcosts.append(round(DPcost,3))
		FCFScosts.append(round(FCFScost,3))
		Ratios.append(round(DPcost/FCFScost,3))
	

	df = pd.DataFrame({
		'DP': DPcosts,
		'FCFS': FCFScosts,
		'runtime': avgRuntimes,
		},index = np.arange(0,1.1,0.1))
	df.index.name = "HV Ratio"
	df.to_csv(RESULTPATH)
