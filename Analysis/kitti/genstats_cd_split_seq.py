
import csv
from statistics import mean

f = open("stats_cd_split_seq.txt", "w")
f = open("stats_cd_split_seq.txt", "a")

for seq in ["00", "02", "05", "06", "07"]:

    ckf = []
    cmp = []
    mkf = []
    mmp = []
    cdsum = []
    cdmean = []
    cdstdev = []
    cdmed = []

    print("seq: {}".format(seq))

    for trial in ["trial0", "trial1", "trial2", "trial3", "trial4"]:

        with open("./traj_exp_split_seq/{}/{}_stats.csv".format(trial, seq)) as fcsv:
            csv_reader = csv.reader(fcsv, delimiter=",")
            next(csv_reader)
            data = next(csv_reader)

            ckf.append(float(data[2]))
            cmp.append(float(data[3]))
            mkf.append(float(data[4]))
            mmp.append(float(data[5]))
            cdsum.append(float(data[7]))
            cdmean.append(float(data[8]))
            cdstdev.append(float(data[9]))
            cdmed.append(float(data[10]))

            print("{}: {:7.2f} {:7.2f} {:7.2f} {:7.2f} {:7.2f} {:7.2f}".format(trial, float(data[2]), float(data[4]), float(data[7]), float(data[8]), float(data[9]), float(data[10])))

    print("mean:   {:7.2f} {:7.2f} {:7.2f} {:7.2f} {:7.2f} {:7.2f}".format(mean(ckf), mean(mkf), mean(cdsum), mean(cdmean), mean(cdstdev), mean(cdmed)))
    print()

    f.write("& {} & {:7.2f} & {:7.2f} & {:7.2f} & {:7.2f} & {:7.2f} & {:7.2f}\n\\\\\n".format(seq, mean(ckf), mean(mkf), mean(cdsum), mean(cdmean), mean(cdstdev), mean(cdmed)))
