# this gets the minimum success velocity or 0 and stores it in max_vels...
import numpy as np
import csv

VELOCITY = [0.2, 0.4, 0.6, 0.8, 1, 1.2, 1.4, 1.6, 1.8, 2]
THRESHOLD = [1, 1.5, 2, 2.5, 3, 3.5, 4]
MU = [0.009, 0.09, 0.05, 0.5, 1]
ANGLE = list(range(0, 180, 15))

removed = []

max_vels = {}  # maximum velocity
min_vels = {}  # minimum fail velocity
all_vels = {}  # a dict that I can sort

for mu in MU:
    for angle in ANGLE:
        for thresh in THRESHOLD:
            index = str(mu) + "_" + str(angle) + "_" + str(thresh)
            all_vels[index] = []
            max_vels[index] = 0
            min_vels[index] = 3  # above maximum velocity to start...

with open('training_data.csv') as csv_file:
    reader = csv.reader(csv_file, delimiter=',')
    for row in reader:
        if row[3] != '-1':
            removed.append(row)

for row in removed:
    mu, angle, thresh = row[1], row[3], row[4] # check
    vel = float(row[2])
    success = int(row[7])
    index = mu + "_" + angle + "_" + thresh

    all_vels[index].append([success, vel])

##for index in all_vels:
##    all_vels[index] = np.array(all_vels[index])
##    sort = np.argsort(all_vels[index][:,1])
##    all_vels[index] = all_vels[index][sort] # sort ascending velocity
##    max_vel = 0
##    for i, run in enumerate(all_vels[index]):
##        if run[0] != 0: max_vel = run[1] # go until failure
##        else: break
##    if i == 0 or i == 1: max_vels[index] = 0
##    else: max_vels[index] = max_vel


# sort each index's runs by ascending velocity
# iterate descending to find first success
# then continue to find next failure
for index in all_vels:
    all_vels[index] = np.array(all_vels[index])
    sort = np.argsort(all_vels[index][:, 1])
    all_vels[index] = all_vels[index][sort]  # sort ascending velocity
    p1 = -1
    p2 = -1
    all_failures = False
    while all_vels[index][p1][0] != 1:  # find first success with p1
        p1 -= 1
        p2 -= 1
        if abs(p1) > len(all_vels[index]):  # all failures case
            all_failures = True
            break
    while not abs(p2) > len(all_vels[index]):  # find next failure with p2
        if all_vels[index][p2][0] != 0:  # find next failure with p2
            p2 -= 1
        else:
            break
    if all_failures == True:
        max_vels[index] = 0
        min_vels[index] = 0
    else:
        max_vels[index] = all_vels[index][p1][1]
        if abs(p2) > len(all_vels[index]):  # if it succeeds all the way down
            min_vels[index] = all_vels[index][0][1]  # min velocity tested (0.2 for now)
        else:
            min_vels[index] = all_vels[index][p2][1]

with open('testing_3.csv', 'w') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerow(('mu', 'angle', 'threshold', 'speed'))
    for index in all_vels.keys():  # inefficient, we could use 1 dict
        mu, angle, thresh = index.split('_')
        writer.writerow([mu, angle, thresh, min_vels[index], max_vels[index]])
