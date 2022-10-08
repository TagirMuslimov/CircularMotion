import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams["figure.autolayout"] = True

df = pd.read_csv('/Users/kmasalimov/anbesp/crazyflie/CircularMotion/20221007-145319_setpos2.csv', header=0, delimiter=';')

cols = df.columns
for col in cols:
    df[col] = df[col].astype(float)

print(df.columns)

df.plot(x='i', y=['px_1'])

plt.show()
