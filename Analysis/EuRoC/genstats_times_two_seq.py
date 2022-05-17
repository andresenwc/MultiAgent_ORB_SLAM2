
import csv
from statistics import mean

f = open("stats_times_two_seq.txt", "w")
f = open("stats_times_two_seq.txt", "a")

for seq1, seq2 in [("MH01", "MH02"), ("MH04", "MH05"), ("V102", "V103"), ("V201", "V202")]:

    kfc = []
    kfm = []
    sim3 = []
    mf = []
    cd = []
    gba = []

    print("seq pair: {}, {}".format(seq1, seq2))

    for trial in ["trial0", "trial1", "trial2", "trial3", "trial4"]:

        with open("./traj_exp_two_seq/{}/{}_{}_stats.csv".format(trial, seq1, seq2)) as fcsv:
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

    f.write("& {} + {} & {:7.2f} & {:7.2f} & {:7.2f} & {:7.2f} & {:7.2f} & {:7.2f}\n\\\\\n".format(seq1, seq2, mean(kfc), mean(kfm), mean(sim3), mean(mf), mean(cd), mean(gba)))
