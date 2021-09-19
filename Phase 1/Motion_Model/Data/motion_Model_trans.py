import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from scipy import stats

trans5 = np.load('Odom_Trans_5.npy')
trans10 =  np.load('Odom_Trans_10.npy')
trans20 =  np.load('Odom_Trans_20.npy')
trans_ang_5 = np.load('Odom_Trans_anglura_5.npy')
trans_ang_10 = np.load('Odom_Trans_anglura_10.npy')
trans_ang_20 = np.load('Odom_Trans_anglura_20.npy')

t = np.linspace(1, len(trans5), len(trans5))
plt.scatter(t, trans5,label="linear translation")
plt.scatter(t, trans_ang_5,label='angular translation')
plt.xlabel("sample number")
plt.ylabel("error of displacement (linear speed)")
plt.legend()
plt.show()
trans_std1 = np.std(trans5)
trans_std2 = np.std(trans10)
trans_std3 = np.std(trans20)

trans_ang_std1 = np.std(trans_ang_5)
trans_ang_std2 = np.std(trans_ang_10)
trans_ang_std3 = np.std(trans_ang_20)



x = np.array([0.05, 0.1, 0.2])
y = np.array([trans_std1, trans_std2, trans_std3])


slope, intercept, r_value, p_value, std_err = stats.linregress(x,y)

t = np.linspace(0,20,1000)
fig = plt.figure()
ax = plt.axes()
ax.plot(t, (slope*t + intercept))
plt.show()

y_angular = np.array([trans_ang_std1, trans_ang_std2, trans_ang_std3])

slope_ang, intercept_ang, r_value_ang, p_value_ang, std_err_ang = stats.linregress(x,y_angular)

fig = plt.figure()
ax = plt.axes()
ax.plot(t, (slope_ang*t + intercept_ang))
plt.show()