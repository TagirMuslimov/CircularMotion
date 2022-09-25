import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True
ax = plt.axes(projection='3d')

headers = ['timestamp', 'range.front', 'range.left', 'range.zrange']

df = pd.read_csv('D:/range2.csv', 
            names=headers, header=0, delimiter=';',
            skip_blank_lines=True,  engine='python'         
            )

cols = df.columns
for col in cols:
    df[col] = df[col].astype(float)
df.set_index('timestamp').plot()


x = df['range.front']
y = df['range.left']
z = df['range.zrange']

plt.cla()

ax.plot3D(x, y, z, 'red')


ax.set_xlabel('range.front, [mm]')
ax.set_ylabel('range.left, [mm]')
ax.set_zlabel('range.zrange, [mm]')


plt.show()

# plt.savefig('position.eps')
