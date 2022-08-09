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
plt.rcParams['figure.figsize'] = [6, 3]
mpl.rcParams['axes.prop_cycle'] = cycler(color=['r', 'y', 'y', 'm'],marker=['D','x','H','8'])


#DP: red line triangle marker
#FCFS: Green line round marker
#MILP: Blue line sqaure marker

df = pd.read_csv(sys.argv[1])
df.plot(x="HV Ratio")

plt.ylabel("Average Waiting Time (second)")
plt.grid()
plt.savefig(sys.argv[1].strip(".csv")+"test.svg", bbox_inches = 'tight')


