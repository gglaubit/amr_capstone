import csv

VELOCITY = [0.2, 0.4, 0.6, 0.8, 1, 1.2, 1.4, 1.6, 1.8, 2]
THRESHOLD = [1, 1.5, 2, 2.5, 3, 3.5, 4]
MU = [0.009, 0.09, 0.05, 0.5, 1]
ANGLE = list(range(0, 180, 15))

full_file = []
removed = []

max_vels = {} # maximum velocity
min_vels = {} # minimum fail velocity

for mu in MU:
    for angle in ANGLE:
        for thresh in THRESHOLD:
            index = str(mu) + "_" + str(angle) + "_" + str(thresh)
            max_vels[index] = 0
            min_vels[index] = 3 # above maximum velocity to start...

with open('training_data.csv') as csv_file:
    reader = csv.reader(csv_file, delimiter=',')
    for row in reader:
        if row[3] != '-1':
            removed.append(row)
        full_file.append(row)

count1 = 0
for row in removed:
    mu, angle, thresh = row[1], row[3], row[4]
    vel = float(row[2])
    success = int(row[5])
    index = mu + "_" + angle + "_" + thresh

    curr_vel = min_vels[index]

    if success != 1 and curr_vel > vel:
        min_vels[index] = vel
    if success == 1:
        count1 += 1

count2 = 0
for row in removed:
    mu, angle, thresh = row[1], row[3], row[4]
    vel = float(row[2])
    success = int(row[5])
    index = mu + "_" + angle + "_" + thresh

    curr_vel = max_vels[index]

    if success == 1 and curr_vel < vel:
        if vel < min_vels[index]:
            max_vels[index] = vel
    if success != 1:
        count2 += 1

with open('testing_2.csv', 'w') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerow(('mu', 'angle', 'threshold', 'speed'))
    for index, vel in max_vels.items():
        mu, angle, thresh = index.split('_')
        writer.writerow([mu, angle, thresh, vel])

print(count1, count2)
