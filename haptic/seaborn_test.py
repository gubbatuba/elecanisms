import csv
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

csvname = 'data/spindowndata_005_12.csv'
df = pd.read_csv(csvname, index_col=0)
df.plot()
plt.show()
# print "plotting..."
# a = sns.tsplot(data=df, time="time", value="ticks", unit="degrees")
# plt.show()
# print "done"
