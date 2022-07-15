from copass import *
from MILPSubproblem import *
import time 
import csv
import pandas as pd

def solveDP(arrivalTime,HV,Copass,P1=1,P2=3):
	zeros =  np.zeros(len(arrivalTime),dtype = 'int')
	table = {}
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

	def PassTime(Pass):
		for i,n in enumerate(Pass):
			if n >= len(HV[i]):
				continue
			if HV[i][n]:
				return P2
		return P1
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
	print(cost)
	return cost,runtime

if __name__ == '__main__':
	meanInterval = 2
	testCount = 50
	DPcosts = []
	FCFScosts = []
	avgRuntimes = []
	Ratios = []
	for HVratio in np.arange(0,1.1,0.1):
		print("ratio:", HVratio)
		FCFScost = 0
		DPcost = 0
		avgRuntime = 0
		successCount = testCount
		i = 0
		while i < testCount:
			arrivalTime,HV,Copass = GenerateTestCase(HVratio,meanInterval,4,10,[])

			cost,runtime = solveDP(arrivalTime,HV,Copass)
			if cost == -1: #gurobi solve fail
				continue

			DPcost += cost 
			t,cost = solveFCFS(arrivalTime,HV,Copass)
			FCFScost += cost
			avgRuntime += runtime
			#t,cost = solveDP(arrivalTime,HV,Copass)
			#DPcost += cost
			np.set_printoptions(precision=2)
			print('FCFS cost:',cost)
			print(t)
			i += 1
		avgRuntime /= testCount
		DPcost /= testCount
		FCFScost /= testCount
		print(DPcost)
		print(FCFScost)
		avgRuntimes.append(round(avgRuntime,3))
		DPcosts.append(round(DPcost,3))
		FCFScosts.append(round(FCFScost,3))
		Ratios.append(round(DPcost/FCFScost,3))
			
	print(DPcosts)
	print(FCFScosts)
	print(avgRuntimes,sum(avgRuntimes)/len(avgRuntimes))
	plt.plot(np.arange(0.0,1.1,0.1),DPcosts,label = "DP")
	plt.plot(np.arange(0.0,1.1,0.1),FCFScosts,label = "FCFS")
	#plt.plot(np.arange(0.0,1.1,0.1),Ratios,label = "N=10")

	plt.xlabel('HVRatio')
	plt.ylabel('Cost(s)')
	plt.legend()
	fig = plt.gcf() 
	fig.savefig('DP_FCFSplot'+ str(meanInterval) + '.svg')
	

	df = pd.DataFrame({
		'DP': DPcosts,
		'FCFS': FCFScosts,
		'Cost ratio': [round(DPcosts[i]/ FCFScosts[i],3) for i in range(len(DPcosts))]
		},index = np.arange(0,1.1,0.1))
	df.index.name = "HV ratio"
	df.to_csv("DP1.csv")
