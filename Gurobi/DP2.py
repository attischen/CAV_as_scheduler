from copass import *
from MILPSubproblem import *
from itertools import product

def solveDP2(arrivalTime,HV,Copass,P1=1,P2=3):
	zeros =  np.zeros(len(arrivalTime),dtype = 'int')
	table = {}
	Nlane = len(arrivalTime)

	def TimeSlice(arrivalTime):
		N = 0
		Tau = []
		for i,a in enumerate(arrivalTime):
			N += len(a)

		for i,a in enumerate(arrivalTime):
			for a_li in a:
				for n1 in range(N):
					for n2 in range(N - n1):
						Tau.append(a_li + n1 * P1 + n2 * P2)

		Tau.append(0)
		Tau = list(set(Tau))
		Tau.sort()
		return Tau



	def MinPass(Pass,lastPassLane,copassT):
		if np.array_equal(Pass,zeros):
			return 0

		if tuple(Pass,lastPassLane,copassT) in table:
			return table[tuple(Pass,lastPassLane,copassT)]
		
		mint = math.inf
		a = arrivalTime[lastPassLane]

		Pass_ = np.array(Pass, copy=True)
		Pass_[lastPassLane] -= 1  
		for i in len(arrivalTime): # lastlastpassing lane
			if Pass_[i] == 0:
				continue

			for tau in Tau:
				t = max(a[Pass_[lastPassLane]],MinPass(Pass_,i,tau) + PassTime(Pass_,lastPassLane,i,tau),Legal(Pass_,lastPassLane))

		if t < mint:
			mint = t
		table[tuple(Pass,lastPassLane,copassT)] = mint
		return mint

	def PassTime(Pass,lastPass,tau):
		if tau == 0: # empty lane
			for i,n in enumerate(Pass):
				if n >= len(HV[i]):
					continue
				if HV[i][n]:
					return P2
		else:
			if HV[lastPass][Pass[lastPass] - 1]:
				return P2
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
				return False 
		return True

	def Initialize():
		for i in range(Nlane):
			Pass = np.zeros(len(arrivalTime),dtype = 'int')
			if not Legal(Pass,i):
				continue 
			Pass[i] += 1
			table[tuple([*Pass,i,0])] = arrivalTime[i][0]

	def NonConflictLane(l1,l2):
		return (l1,l2) in Copass or (l2,l1) in Copass

	N = [len(a) for a in arrivalTime]
	Ns = [list(range(len(a) + 1)) for a in arrivalTime]
	Tau = TimeSlice(arrivalTime)
	mincost = math.inf
	Initialize()
	#print(table)

	for Pass in product(*Ns):    
		#print(Pass) 
		if sum(list(Pass)) <= 1: # 0,0,0,0 or 0,0,0,1...
			continue
		
		for g in range(Nlane):
			if Pass[g] == 0:
				continue
			Pass_ = np.array(Pass, copy=True)
			Pass_[g] -= 1
			if not Legal(Pass_,g):
				continue

			for tau in Tau:
				mint = math.inf 
				
				#state: Pass_,g_,tau_ -> Pass,g,tau
				if tau == 0: # intersection is clear when entering 
					for g_ in range(Nlane):
						for tau_ in Tau: 
							if tuple([*Pass_,g_,tau_]) not in table or table[tuple([*Pass_,g_,tau_])] == math.inf:
								continue
							t,t_ = None,table[tuple([*Pass_,g_,tau_])]
							t = max(t_ + PassTime(Pass_,g_,tau_), arrivalTime[g][Pass_[g]],tau_)
							if t < mint:
								mint = t
				else:
					for g_ in range(Nlane):
						if not NonConflictLane(g_,g):
							continue

						for tau_ in Tau: 
							if tuple([*Pass_,g_,tau_]) not in table or table[tuple([*Pass_,g_,tau_])] == math.inf:
								continue
							t,t_ = None,table[tuple([*Pass_,g_,tau_])]
							if tau == t_ + PassTime(Pass_,g_,tau_)  and NonConflictLane(g_,g):   # using copass 
								t = max(tau_ , arrivalTime[g][Pass_[g]],t_) 
							else: 
								continue 
							if t < mint:
								mint = t

				


				table[tuple([*Pass,g,tau])] = mint
				if mint != math.inf:
					#print(Pass,mint)
					pass

				if np.array_equal(Pass,N):
					if mint < mincost: 
						mincost = mint

	

	for k,v in table.items():
		if v is not math.inf:
			print(k)
			print("   ",v)
	return mincost
	

if __name__ == '__main__':
	meanInterval = 1
	testCount = 1
	DPcosts = []
	FCFScosts = []
	MILPcosts = []
	Ratios = []
	np.set_printoptions(precision=3)
	for HVratio in np.arange(0.0,1.1,0.1):
		print("ratio:", HVratio)
		FCFScost = 0
		DPcost = 0
		MILPcost = 0
		successCount = testCount
		i = 0
		while i < testCount:
			arrivalTime,HV,Copass = GenerateTestCase(HVratio,meanInterval,4,2,[(0,1),(2,3)])
			#print("Arrival Time:",arrivalTime)
			t,cost,runtime = solveMILP(arrivalTime,HV,Copass)
			MILPcost += cost
			print('MILP cost:{},runtime:{}'.format(cost,runtime))
			#print(t)
			if cost == -1: #gurobi solve fail
				continue

			DPcost += cost 
			t,cost = solveFCFS(arrivalTime,HV,Copass)
			FCFScost += cost
			print('FCFS cost:',cost)
			#print(t)

			cost = solveDP2(arrivalTime,HV,Copass)
			print("DP cost:",cost)
			i += 1


		DPcost /= testCount
		FCFScost /= testCount
		print(DPcost)
		print(FCFScost)
		DPcosts.append(round(DPcost,3))
		FCFScosts.append(round(FCFScost,3))
		Ratios.append(round(DPcost/FCFScost,3))
			
	print(DPcosts)
	print(FCFScosts)

	#plt.plot(np.arange(0.0,1.1,0.1),MILPcosts,label = "MILP")
	#plt.plot(np.arange(0.0,1.1,0.1),FCFScosts,label = "FCFS")
	plt.plot(np.arange(0.0,1.1,0.1),Ratios,label = "N=10")

	plt.xlabel('HVRatio')
	plt.ylabel('MILP/FCFS')
	plt.legend()
	fig = plt.gcf() 
	fig.savefig('DPplot'+ str(meanInterval) + '.png')