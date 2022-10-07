import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

headers = ['i', 'T_Z', 'v_z', 'CX', 'CY', 'k', 'R', 'D_12', 'v_f', 'v_cruis', 'k_f', 'p12', 'px_1', 'py_1', 'pz_1', 'd_1', 'phi_1', 'angle_1', 'v1', 'vx1', 'vy1', 'setPx1', 'setPy1', 'px_2', 'py_2', 'pz_2', 'd_2', 'phi_2', 'angle_2', 'v2', 'vx2', 'vy2', 'setPx2', 'setPy2']
df = pd.read_csv('20221007-133730_setpos2.csv', names=headers, header=0, delimiter=';')

cols = df.columns
for col in cols:
    df[col] = df[col].astype(float)

df.plot(x='vx1', y='vy1')

plt.show()
