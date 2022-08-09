import sys
import matplotlib.pyplot as plt
import pandas as pd 
import matplotlib as mpl
from cycler import cycler

mpl.rcParams['lines.linewidth'] = 1
mpl.rcParams['lines.linestyle'] = '-'
mpl.rcParams['lines.marker'] = 's'
mpl.rcParams['lines.markersize'] = 6
mpl.rcParams['font.size'] = 14
mpl.rcParams['font.family'] =  'Times New Roman'
plt.rcParams['figure.figsize'] = [6, 4]
mpl.rcParams['axes.prop_cycle'] = cycler(color=['r', 'y', 'y', 'm'],marker=['D','x','H','8'])
plt.rcParams['text.usetex'] = True

#DP: red line triangle marker
#FCFS: Green line round marker
#MILP: Blue line sqaure marker


df = pd.read_csv(sys.argv[1])
#df.columns=['HV Ratio',"$\it{N}$ = 12 (No Subproblem)","$\it{N}$ = 36 (Subproblem Size = 12)"]
#df.columns=['HV Ratio','FCFS','Subcase size=4','Subcase size=12','Subcase size=20']
df.plot(x="HV Ratio")

#plt.ylabel("Objective (second)")
plt.ylabel("Objective Ratio to FCFS")
plt.grid()
plt.savefig(sys.argv[1].strip(".csv")+"test.svg", bbox_inches = 'tight')


