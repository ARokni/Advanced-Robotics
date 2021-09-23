import numpy as np

robot_max_range = 0.40000000596
file_10cm_y = open('Data/sample_range_10cm_y.txt', 'r')
lines_10cm_y = file_10cm_y.readlines()
measurments_10cm_y = []
for line in lines_10cm_y:
    measurments_10cm_y.append(float(line.strip()))

file_20cm_y = open('Data/sample_range_20cm_y.txt', 'r')
lines_20cm_y = file_20cm_y.readlines()
measurments_20cm_y = []
for line in lines_20cm_y:
    measurments_20cm_y.append(float(line.strip()))

file_30cm_y = open('Data/sample_range_30cm_y.txt', 'r')
lines_30cm_y = file_30cm_y.readlines()
measurments_30cm_y = []
for line in lines_30cm_y:
    measurments_30cm_y.append(float(line.strip()))

file_30cm_x = open('Data/sample_range_30cm_x.txt', 'r')
lines_30cm_x = file_30cm_x.readlines()
measurments_30cm_x = []
for line in lines_30cm_x:
    measurments_30cm_x.append(float(line.strip()))

true_10cm = 0.100000000000
true_20cm = 0.200000000000
true_30cm = 0.300000000000
sigma_hit_10cm_y = np.std(np.array(measurments_10cm_y))
sigma_hit_20cm_y = np.std(np.array(measurments_20cm_y))
sigma_hit_30cm_y = np.std(np.array(measurments_30cm_y))
sigma_hit_30cm_x = np.std(np.array(measurments_30cm_x))

print("sigma hit ---> robot in 10cm distance: ", sigma_hit_10cm_y)
print("sigma hit ---> robot in 20cm distance: ", sigma_hit_20cm_y)
print("sigma hit ---> robot in 30cm distance: ", sigma_hit_30cm_y)
# print("sigma hit in 30 cm in x-axis: ", sigma_hit_30cm_x)

sigma_hit = (sigma_hit_10cm_y + sigma_hit_20cm_y + sigma_hit_30cm_y + sigma_hit_30cm_x) / 4
print("initial sigma_hit: ", sigma_hit)


def p_rand(x):
    if x <= robot_max_range:
        p = 1 / robot_max_range
    else:
        p = 0
    return p


def p_max(x):
    if x == robot_max_range:
        p = 1
    else:
        p = 0
    return p


def p_hit(x1, x2):
    num = np.exp(-0.5 * np.square((x1 - x2) / sigma_hit))
    p = num / np.sqrt(2 * np.pi * np.square(sigma_hit))
    tot = 1
    if 0 <= x2 <= robot_max_range:
        return p * (1 / tot)
    else:
        return 0


def learn_intrinsic_parameters(measurements, true_range):
    iteration = 0
    num_of_iteration = 100
    e_hit = np.zeros((len(measurements)))
    e_max = np.zeros((len(measurements)))
    e_rand = np.zeros((len(measurements)))
    z_hit = 0
    z_max = 0
    z_rand = 0
    while iteration < num_of_iteration:
        z_hit_before = z_hit
        z_rand_before = z_rand
        for measure in range(len(measurements)):
            eta = 1 / (p_hit(true_range, measurements[measure]) + p_max(measurements[measure]) + p_rand(measurements[measure]))
            e_hit[measure] = eta * p_hit(true_range, measurements[measure])
            e_max[measure] = eta * p_max(measurements[measure])
            e_rand[measure] = eta * p_rand(measurements[measure])
        z_hit = e_hit.mean()
        z_max = e_max.mean()
        z_rand = e_rand.mean()
        if z_hit - z_hit_before < 1e-6 and z_rand - z_rand_before < 1e-6:
            return z_hit, z_max, z_rand


print("\nlearned parameters (z_hit, z_max, z_rand) in 10cm: ",learn_intrinsic_parameters(np.array(measurments_10cm_y), true_10cm))
print("\nlearned parameters (z_hit, z_max, z_rand) in 20cm: ",learn_intrinsic_parameters(np.array(measurments_20cm_y), true_20cm))
print("\nlearned parameters (z_hit, z_max, z_rand) in 30cm: ",learn_intrinsic_parameters(np.array(measurments_30cm_y), true_30cm))



import matplotlib.pyplot as plt
plt.figure(figsize=(10,10))
x = np.linspace(0, 1000, 1000)
plt.scatter(x,measurments_10cm_y, c='pink', marker='.',label='distance to robot = 10cm')
plt.scatter(500,true_10cm, c='k',marker='x',s=200)

plt.scatter(x,measurments_20cm_y,c='cyan', marker='.',label='distance to robot = 20cm')
plt.scatter(500,true_20cm, c='k',marker='x',s=200)

plt.scatter(x,measurments_30cm_y,c='yellow', marker='.',label='distance to robot = 30cm')
plt.scatter(500,true_30cm, c='k',marker='x',s=200)
plt.xlabel("sample number")
plt.ylabel("laser range")
plt.legend()

plt.show()
