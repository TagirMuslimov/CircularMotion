import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

headers = ['i', 'x', 'y', 'd', 'phi', 'angle', 'vx', 'vy', 'CX', 'CY', 'k', 'R', 'v']

df = pd.read_csv('1.csv', names=headers, header=0, delimiter=';')

cols = df.columns
for col in cols:
    df[col] = df[col].astype(float)

df.plot(x='i', y=['x', 'y'])

plt.show()
