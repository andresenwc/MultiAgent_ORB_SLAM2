import copy

from evo.core import metrics, sync
from evo.tools import file_interface

from statistics import mean

# seq_exp - sequence
# seq_gt - gt sequence
# trial - trial string
def gen_stats(seq_exp: str, seq_gt: str, trial: str):

    # ground truth trajectory
    tgt = file_interface.read_euroc_csv_trajectory("./traj_gt/{}.csv".format(seq_gt))
    file_interface.write_tum_trajectory_file("./traj_gt/{}_tum.csv".format(seq_gt), tgt, confirm_overwrite=False)
    tgt = file_interface.read_tum_trajectory_file("./traj_gt/{}_tum.csv".format(seq_gt))

    # experimental trajectories
    tos2 = file_interface.read_tum_trajectory_file("./traj_os2/{}/{}.txt".format(trial, seq_gt))
    texp = file_interface.read_tum_trajectory_file("./traj_exp_two_seq/{}/{}.txt".format(trial, seq_exp))

    # align and scale the trajectories
    tos2, tgt_os2 = sync.associate_trajectories(tos2, tgt)
    tos2_as = copy.deepcopy(tos2)
    tos2_as.align(tgt_os2, correct_scale=True)

    texp, tgt_exp = sync.associate_trajectories(texp, tgt)
    texp_as = copy.deepcopy(texp)
    texp_as.align(tgt_exp, correct_scale=True)

    # Some setup
    t_met = metrics.PoseRelation.translation_part
    r_met = metrics.PoseRelation.rotation_angle_deg
    delta = 1
    delta_unit = metrics.Unit.meters
    stat_type = metrics.StatisticsType.mean

    # compute APE translation
    at_os2_metric = metrics.APE(t_met)
    at_os2_metric.process_data((tgt_os2, tos2_as))
    at_os2_stat = at_os2_metric.get_statistic(stat_type)

    at_exp_metric = metrics.APE(t_met)
    at_exp_metric.process_data((tgt_exp, texp_as))
    at_exp_stat = at_exp_metric.get_statistic(stat_type)

    # compute RPE translation / rotation
    rt_os2_metric = metrics.RPE(t_met, delta, delta_unit)
    rt_os2_metric.process_data((tgt_os2, tos2_as))
    rt_os2_stat = rt_os2_metric.get_statistic(stat_type)

    rt_exp_metric = metrics.RPE(t_met, delta, delta_unit)
    rt_exp_metric.process_data((tgt_exp, texp_as))
    rt_exp_stat = rt_exp_metric.get_statistic(stat_type)

    rr_os2_metric = metrics.RPE(r_met, delta, delta_unit)
    rr_os2_metric.process_data((tgt_os2, tos2_as))
    rr_os2_stat = rr_os2_metric.get_statistic(stat_type)

    rr_exp_metric = metrics.RPE(r_met, delta, delta_unit)
    rr_exp_metric.process_data((tgt_exp, texp_as))
    rr_exp_stat = rr_exp_metric.get_statistic(stat_type)

    # done
    return [at_exp_stat, rt_exp_stat, rr_exp_stat, at_os2_stat, rt_os2_stat, rr_os2_stat]

f = open("stats_two_seq.txt", "w")
f = open("stats_two_seq.txt", "a")

for seq1, seq2 in [("MH01", "MH02"), ("MH04", "MH05"), ("V102", "V103"), ("V201", "V202")]:

    at_exp_stats = [[], []]
    rt_exp_stats = [[], []]
    rr_exp_stats = [[], []]
    at_os2_stats = [[], []]
    rt_os2_stats = [[], []]
    rr_os2_stats = [[], []]

    print("seq pair: {}, {}".format(seq1, seq2))

    for trial in ["trial0", "trial1", "trial2", "trial3", "trial4"]:

        stats1 = gen_stats("{}_{}_{}".format(seq1, seq2, seq1), seq1, trial)
        stats2 = gen_stats("{}_{}_{}".format(seq1, seq2, seq2), seq2, trial)

        at_exp_stats[0].append(stats1[0])
        rt_exp_stats[0].append(stats1[1])
        rr_exp_stats[0].append(stats1[2])
        at_os2_stats[0].append(stats1[3])
        rt_os2_stats[0].append(stats1[4])
        rr_os2_stats[0].append(stats1[5])
        
        at_exp_stats[1].append(stats2[0])
        rt_exp_stats[1].append(stats2[1])
        rr_exp_stats[1].append(stats2[2])
        at_os2_stats[1].append(stats2[3])
        rt_os2_stats[1].append(stats2[4])
        rr_os2_stats[1].append(stats2[5])

        print("{}, {}: {:5.2f} {:5.2f} {:5.2f} {:5.2f} {:5.2f} {:5.2f}".format(trial, seq1, stats1[0], stats1[1], stats1[2], stats1[3], stats1[4], stats1[5]))
        print("{}, {}: {:5.2f} {:5.2f} {:5.2f} {:5.2f} {:5.2f} {:5.2f}".format(trial, seq2, stats2[0], stats2[1], stats2[2], stats2[3], stats2[4], stats2[5]))

    print("mean, {}:   {:5.2f} {:5.2f} {:5.2f} {:5.2f} {:5.2f} {:5.2f}".format(seq1, mean(at_exp_stats[0]), mean(rt_exp_stats[0]), mean(rr_exp_stats[0]), mean(at_os2_stats[0]), mean(rt_os2_stats[0]), mean(rr_os2_stats[0])))

    print("mean, {}:   {:5.2f} {:5.2f} {:5.2f} {:5.2f} {:5.2f} {:5.2f}".format(seq2, mean(at_exp_stats[1]), mean(rt_exp_stats[1]), mean(rr_exp_stats[1]), mean(at_os2_stats[1]), mean(rt_os2_stats[1]), mean(rr_os2_stats[1])))
    print()

    f.write("& {} & {:5.2f} & {:5.2f} & {:5.2f} & {:5.2f} & {:5.2f} & {:5.2f}\n\\\\\n".format(seq1, mean(at_exp_stats[0]), mean(rt_exp_stats[0]), mean(rr_exp_stats[0]), mean(at_os2_stats[0]), mean(rt_os2_stats[0]), mean(rr_os2_stats[0])))

    f.write("& {} & {:5.2f} & {:5.2f} & {:5.2f} & {:5.2f} & {:5.2f} & {:5.2f}\n\\\\\n".format(seq2, mean(at_exp_stats[1]), mean(rt_exp_stats[1]), mean(rr_exp_stats[1]), mean(at_os2_stats[1]), mean(rt_os2_stats[1]), mean(rr_os2_stats[1])))
