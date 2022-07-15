import sys
import matplotlib.pyplot as plt
import pandas as pd 
import matplotlib as mpl
from cycler import cycler

mpl.rcParams['lines.linewidth'] = 1
mpl.rcParams['lines.linestyle'] = '-'
mpl.rcParams['lines.marker'] = 's'
mpl.rcParams['lines.markersize'] = 5
plt.rcParams['figure.figsize'] = [6, 4]
mpl.rcParams['axes.prop_cycle'] = cycler(color=['r', 'b', 'g', 'y'])

df = pd.read_csv(sys.argv[1])
#df.columns=['HV ratio','$N_i$=4 (No subcasing)','$N_i$=10']
df.plot(x="HV ratio")

plt.ylabel("Cost (second)")
#plt.ylabel("Cost ratio to FCFS")
plt.grid()
plt.savefig(sys.argv[1].strip(".csv")+".svg")


