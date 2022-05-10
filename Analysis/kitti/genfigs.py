import copy

import evo.core.lie_algebra as lie
from evo.tools import plot, file_interface

import numpy as np
from matplotlib.axis import Axis
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})

# seq - sequence
# n - number of agents
def plot_traj(seq: str, n: int):

    colors = ["blue", "green", "red", "yellow"]

    # ground truth trajectory
    tgt = file_interface.read_kitti_poses_file("./traj_gt/{}.txt".format(seq))

    # experimental trajectories
    tec = file_interface.read_kitti_poses_file("./traj_exp/{}.txt".format(seq))
    te = []
    for i in range(n):
        fname = "./traj_exp/{}_{}.txt".format(seq, i)
        te.append(file_interface.read_kitti_poses_file(fname))

    # align and scale the trajectories
    tec_as = copy.deepcopy(tec)
    r_a, t_a, s = tec_as.align(tgt, correct_scale=True)

    te_as = []
    for t in te:
        t_as = copy.deepcopy(t)
        t_as.scale(s)
        t_as.transform(lie.se3(r_a, t_a))
        te_as.append(t_as)
    
    # plot the trajectories, save the plot
    fig = plt.figure()
    plot_mode = plot.PlotMode.xz

    ax = plot.prepare_axis(fig, plot_mode, subplot_arg=111)
    plot.traj(ax, plot_mode, tgt, "--", "gray", "gt")

    for i in range(len(te_as)):
        plot.traj(ax, plot_mode, te_as[i], "-", colors[i], "cam{}".format(i))

    lgd = ax.legend(loc="best", prop={"size" : 12})
    fig.axes.append(ax)
    plt.title("KITTI {} Trajectory Comparison".format(seq))

    # asp = np.diff(ax.get_xlim())[0] / np.diff(ax.get_ylim())[0]
    # ax.set_aspect(asp)

    fig.tight_layout()
    plt.savefig("figs/KITTI_{}.pgf".format(seq), bbox_extra_artists=(lgd,), bbox_inches="tight")

plot_traj("00", 2)
plot_traj("02", 2)
plot_traj("05", 2)
plot_traj("06", 2)
plot_traj("07", 2)
