import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from scipy import stats

trans_ang_5 = np.load('Odom_Angular_15.npy')*180/np.pi
trans_ang_10 =  np.load('Odom_Angular_20.npy')*180/np.pi
trans_ang_20 =  np.load('Odom_Angular_30.npy')*180/np.pi
trans5 = np.load('Odom_Angular_dist_15.npy')
trans10 = np.load('Odom_Angular_dist_20.npy')
trans20 = np.load('Odom_Angular_dist_30.npy')


t = np.linspace(1, len(trans5), len(trans5))
plt.scatter(t, trans5,label="linear translation")
plt.scatter(t, trans_ang_5,label='angular translation')
plt.xlabel("sample number")
plt.ylabel("error of displacement (angular speed)")
plt.legend(loc=0)
plt.show()
trans_std1 = np.std(trans5)
trans_std2 = np.std(trans10)
trans_std3 = np.std(trans20)

trans_ang_std1 = np.std(trans_ang_5)
trans_ang_std2 = np.std(trans_ang_10)
trans_ang_std3 = np.std(trans_ang_20)



x = np.array([5, 10, 20])
y = np.array([trans_std1, trans_std2, trans_std3])

print("alpha1:",np.mean(y/x))
slope, intercept, r_value, p_value, std_err = stats.linregress(x,y)

t = np.linspace(0,1,1000)
fig = plt.figure()
ax = plt.axes()
ax.plot(t, (slope*t + intercept))

y_angular = np.array([trans_ang_std1, trans_ang_std2, trans_ang_std3])
print("alpha4:",np.mean(y_angular/x))

slope_ang, intercept_ang, r_value_ang, p_value_ang, std_err_ang = stats.linregress(x,y_angular)

fig = plt.figure()
ax = plt.axes()
ax.plot(t, (slope_ang*t + intercept_ang))