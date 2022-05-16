import copy

from evo.core import metrics, sync
from evo.tools import file_interface

from statistics import mean

# seq - sequence
def gen_stats(seq: str, trial: str):

    # ground truth trajectory
    tgt = file_interface.read_tum_trajectory_file("./traj_gt/{}.txt".format(seq))

    # experimental trajectories
    tos2 = file_interface.read_tum_trajectory_file("./traj_os2/{}/{}.txt".format(trial, seq))
    texp = file_interface.read_tum_trajectory_file("./traj_exp_split_seq/{}/{}.txt".format(trial, seq))

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

f = open("stats_split_seq.txt", "w")
f = open("stats_split_seq.txt", "a")

for seq in ["fr1_desk", "fr1_desk2", "fr1_room", "fr2_xyz", "fr3_office"]:

    at_exp_stats = []
    rt_exp_stats = []
    rr_exp_stats = []
    at_os2_stats = []
    rt_os2_stats = []
    rr_os2_stats = []

    print("seq: {}".format(seq))

    for trial in ["trial0", "trial1", "trial2", "trial3", "trial4"]:

        stats = gen_stats(seq, trial)

        at_exp_stats.append(stats[0])
        rt_exp_stats.append(stats[1])
        rr_exp_stats.append(stats[2])
        at_os2_stats.append(stats[3])
        rt_os2_stats.append(stats[4])
        rr_os2_stats.append(stats[5])

        print("{}: {:5.2f} {:5.2f} {:5.2f} {:5.2f} {:5.2f} {:5.2f}".format(trial, stats[0], stats[1], stats[2], stats[3], stats[4], stats[5]))

    print("mean:   {:5.2f} {:5.2f} {:5.2f} {:5.2f} {:5.2f} {:5.2f}".format(mean(at_exp_stats), mean(rt_exp_stats), mean(rr_exp_stats), mean(at_os2_stats), mean(rt_os2_stats), mean(rr_os2_stats)))
    print()

    f.write("& {} & {:5.2f} & {:5.2f} & {:5.2f} & {:5.2f} & {:5.2f} & {:5.2f}\n\\\\\n".format(seq, mean(at_exp_stats), mean(rt_exp_stats), mean(rr_exp_stats), mean(at_os2_stats), mean(rt_os2_stats), mean(rr_os2_stats)))
