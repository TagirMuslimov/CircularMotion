import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams["figure.autolayout"] = True

df = pd.read_csv('/home/tagir/CircularMotion/20221217-113451_setpos.csv', header=0, delimiter=';')

cols = df.columns
for col in cols:
    df[col] = df[col].astype(float)

print(df.columns)

# df.plot(x='i', y=['setPx1','setPx2','setPx3'])
# df.plot(x='i', y=['setPy1','setPy2','setPy3'])
df.plot(x='i', y=['p12', 'p23'])
df.plot(x='i', y=['d_1','d_2', 'd_3'])
df.plot(x='setPx1', y=['setPy1'])
df.plot(x='setPx2', y=['setPy2'])
# df.plot(x='setPx3', y=['setPy3'])
plt.show()
