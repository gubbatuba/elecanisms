import csv
import pandas as pd
import seaborn as sns

csvname = '/home/reggert/Documents/elecanisms/haptic/spindowndata_005_slowmotor.csv'
df = pd.read_csv(csvname)
plot_data = sns.load_dataset(df)
sns.tsplot(data=plot_data, time="time", value="ticks")