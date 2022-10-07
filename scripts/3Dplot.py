import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True
ax = plt.axes(projection='3d')

headers = ['i', 'T_Z', 'v_z', 'CX', 'CY', 'k', 'R', 'D_12', 'v_f', 'v_cruis', 'k_f', 'p12', 'px_1', 'py_1', 'pz_1', 'd_1', 'phi_1', 'angle_1', 'v1', 'vx1', 'vy1', 'setPx1', 'setPy1', 'px_2', 'py_2', 'pz_2', 'd_2', 'phi_2', 'angle_2', 'v2', 'vx2', 'vy2', 'setPx2', 'setPy2']

df = pd.read_csv('20221007-145319_setpos2.csv', 
            names=headers, header=0, delimiter=';',
            skip_blank_lines=True,  engine='python'         
            )

cols = df.columns
for col in cols:
    df[col] = df[col].astype(float)
df.set_index('i').plot()


x = df['px_1']
y = df['py_1']
z = df['pz_1']

plt.cla()

ax.plot3D(x, y, z, 'red')


ax.set_xlabel('range.front, [mm]')
ax.set_ylabel('range.left, [mm]')
ax.set_zlabel('range.zrange, [mm]')


plt.show()

# plt.savefig('position.eps')
