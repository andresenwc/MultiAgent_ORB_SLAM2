import copy
import logging
import sys

import evo.core.lie_algebra as lie
from evo.core import trajectory
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
traj_00 = file_interface.read_kitti_poses_file("./traj_exp/00.txt")
traj_00_1 = file_interface.read_kitti_poses_file("./traj_exp/00_1.txt")
traj_00_2 = file_interface.read_kitti_poses_file("./traj_exp/00_2.txt")

# ground truth trajectory
traj_00_gt = file_interface.read_kitti_poses_file("./traj_gt/00.txt")

# align trajectories to gt trajectory
traj_00_as = copy.deepcopy(traj_00)
r_a, t_a, s = traj_00_as.align(traj_00_gt, correct_scale=True)

traj_00_1_as = copy.deepcopy(traj_00_1)
traj_00_1_as.scale(s)
traj_00_1_as.transform(lie.se3(r_a, t_a))

traj_00_2_as = copy.deepcopy(traj_00_2)
traj_00_2_as.scale(s)
traj_00_2_as.transform(lie.se3(r_a, t_a))

fig = plt.figure()
plot_mode = plot.PlotMode.xz

ax = plot.prepare_axis(fig, plot_mode, subplot_arg=111)
plot.traj(ax, plot_mode, traj_00_gt, "--", "gray", "gt")
plot.traj(ax, plot_mode, traj_00_1_as, "-", "blue", "cam1")
plot.traj(ax, plot_mode, traj_00_2_as, "-", "green", "cam2")
ax.legend(loc="upper left", prop={"size" : 12})
fig.axes.append(ax)
plt.title("KITTI 00 Trajectory Comparison")

fig.tight_layout()
plt.savefig("figs/KITTI_00.pgf", bbox_inches="tight")
