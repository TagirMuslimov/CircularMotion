import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams["figure.autolayout"] = True

df = pd.read_csv('/home/tagir/CircularMotion/20221008-163951_setpos2.csv', header=0, delimiter=';')

cols = df.columns
for col in cols:
    df[col] = df[col].astype(float)

print(df.columns)

# df.plot(x='i', y=['px_1','px_2','py_1','py_2'])
df.plot(x='i', y=['px_1'])


plt.show()
