import copy
import logging
import sys

import evo.core.lie_algebra as lie
from evo.core import trajectory, sync
from evo.tools import plot, file_interface, log

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

# experimental trajectories
traj_00 = file_interface.read_tum_trajectory_file("./traj_exp/V101.txt")
traj_00_0 = file_interface.read_tum_trajectory_file("./traj_exp/V101_0.txt")
traj_00_1 = file_interface.read_tum_trajectory_file("./traj_exp/V101_1.txt")
traj_00_2 = file_interface.read_tum_trajectory_file("./traj_exp/V101_2.txt")
traj_00_3 = file_interface.read_tum_trajectory_file("./traj_exp/V101_3.txt")

# ground truth trajectory
traj_00_gt = file_interface.read_euroc_csv_trajectory("./traj_gt/V101.csv")
file_interface.write_tum_trajectory_file("./traj_gt/V101_tum.csv", traj_00_gt, confirm_overwrite=False)
traj_00_gt = file_interface.read_tum_trajectory_file("./traj_gt/V101_tum.csv")

# align trajectories to gt trajectory
traj_00, traj_00_gt = sync.associate_trajectories(traj_00, traj_00_gt)
traj_00_as = copy.deepcopy(traj_00)
r_a, t_a, s = traj_00_as.align(traj_00_gt, correct_scale=True)


traj_00_0_as = copy.deepcopy(traj_00_0)
traj_00_0_as.scale(s)
traj_00_0_as.transform(lie.se3(r_a, t_a))

traj_00_1_as = copy.deepcopy(traj_00_1)
traj_00_1_as.scale(s)
traj_00_1_as.transform(lie.se3(r_a, t_a))

traj_00_2_as = copy.deepcopy(traj_00_2)
traj_00_2_as.scale(s)
traj_00_2_as.transform(lie.se3(r_a, t_a))

traj_00_3_as = copy.deepcopy(traj_00_3)
traj_00_3_as.scale(s)
traj_00_3_as.transform(lie.se3(r_a, t_a))

fig = plt.figure()
plot_mode = plot.PlotMode.xyz

ax = plot.prepare_axis(fig, plot_mode, subplot_arg=111)
plot.traj(ax, plot_mode, traj_00_gt, "--", "gray", "gt")
plot.traj(ax, plot_mode, traj_00_0_as, "-", "blue", "cam0")
plot.traj(ax, plot_mode, traj_00_1_as, "-", "green", "cam1")
plot.traj(ax, plot_mode, traj_00_2_as, "-", "red", "cam2")
plot.traj(ax, plot_mode, traj_00_3_as, "-", "yellow", "cam3")
ax.legend(loc="upper left", prop={"size" : 12})
fig.axes.append(ax)
plt.title("EuRoC V101 Trajectory Comparison")

fig.tight_layout()
plt.savefig("figs/EuRoC_V101.pgf", bbox_inches="tight")
