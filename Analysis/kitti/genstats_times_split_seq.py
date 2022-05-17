
import csv
from statistics import mean

f = open("stats_times_split_seq.txt", "w")
f = open("stats_times_split_seq.txt", "a")

for seq in ["00", "02", "05", "06", "07"]:

    kfc = []
    kfm = []
    sim3 = []
    mf = []
    cd = []
    gba = []

    print("seq: {}".format(seq))

    for trial in ["trial0", "trial1", "trial2", "trial3", "trial4"]:

        with open("./traj_exp_split_seq/{}/{}_stats.csv".format(trial, seq)) as fcsv:
            csv_reader = csv.reader(fcsv, delimiter=",")
            next(csv_reader)
            data = next(csv_reader)

            kfc.append(float(data[2]))
            kfm.append(float(data[4]))
            sim3.append(float(data[0])/1000)
            mf.append(float(data[1])/1000)
            cd.append(float(data[6])/1000)
            gba.append(float(data[11])/1000)

            print("{}: {:7.2f} {:7.2f} {:7.2f} {:7.2f} {:7.2f} {:7.2f}".format(trial, float(data[2]), float(data[4]), float(data[0])/1000, float(data[1])/1000, float(data[6])/1000, float(data[11])/1000))

    print("mean:   {:7.2f} {:7.2f} {:7.2f} {:7.2f} {:7.2f} {:7.2f}".format(mean(kfc), mean(kfm), mean(sim3), mean(mf), mean(cd), mean(gba)))
    print()

    f.write("& {} & {:7.2f} & {:7.2f} & {:7.2f} & {:7.2f} & {:7.2f} & {:7.2f}\n\\\\\n".format(seq, mean(kfc), mean(kfm), mean(sim3), mean(mf), mean(cd), mean(gba)))
