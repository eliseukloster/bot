print('Use python3 ezuplot.py -h to display a help message.')
import math
from subprocess import call
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os

# Parser will be useful to change plot and simulation parameters.
parser = argparse.ArgumentParser()

# Extension
parser.add_argument('-e', '--extension',
    help = 'Sets the export format of the plot. Defaults to png.',
    choices = ['png', 'jpg', 'pgf', 'pdf', 'svg'],
    default = 'png'
    )
# Resolution (dpi)
parser.add_argument('-r', '--resolution',
    help = 'Sets the resolution of the plot in dpi. Defaults to 300.',
    type = int,
    default = 300
    )
# Output file name
parser.add_argument('-o', '--output',
    help = 'Sets the output file name. Defaults to plot.',
    type = str,
    default = 'plot'
    )
# Title
parser.add_argument('-t', '--title',
    help = 'Sets the title of the plot.',
    type = str
    )
# X label
parser.add_argument('-x', '--xlabel',
    help = 'Sets the x label of the plot.',
    type = str
    )
# Y label
parser.add_argument('-y', '--ylabel',
    help = 'Sets the y label of the plot.',
    type = str
    )
# Data file name
parser.add_argument('-d', '--data',
    help = 'Sets the input data file name. Defaults to data.csv',
    type = str,
    default = 'data.csv')
# Scatter plot
parser.add_argument('-s', '--scatter',
    help = 'Sets the plot type to scatter plot.',
    action = 'store_true')
# Modify file
parser.add_argument('-c', '--custom',
    help = 'Sets the customization file name. \
    Please do not include the .py extension. \
    A template is generated if the file doesn\'t exist.')

args = parser.parse_args()

# ======================
# DEFAULT CONFIGURATIONS
# ======================

file_name = args.output
title = args.title
xlabel = args.xlabel
ylabel = args.ylabel
resolution = args.resolution
data = np.load(args.data)
scatter = args.scatter

bbox_inches = 'tight'
figsize_inches = (6.4, 4.8)

# ======================
# CUSTOMIZATION TEMPLATE
# ======================

custom_default = """
# ============
# EXTRACT DATA
# ============

def get_data(data):
    xs = data[1:, 0]
    ys = data[1:, 1]
    arrays = [xs, ys]
    return arrays

# ==============
# CUSTOMIZE PLOT
# ==============

def modify(fig, ax):
    return fig, ax

# ========================
# CUSTOMIZE plot arguments
# ========================

def kwargs(arrays, fig, ax):
    kwargs = {
        'c': '#000000',
        'linewidth': 0.75,
    }
    return kwargs
"""

# ===================
# BEAUTIFUL FUNCTIONS
# ===================
def file_exists(name, ext):
    return os.path.exists(f'{name}.{ext}')
def savefigure(fig, name, ext):
    i=1
    while(os.path.exists(f'{name}{i}.{ext}')): i+=1
    fig.savefig(f'{name}{i}.{ext}',
        dpi=resolution,
        bbox_inches=bbox_inches
        )
    print(f'Figure saved: {name}{i}.{ext}')
def generateplot(figsize):
    # fig, ax
    fig, ax = plt.subplots(figsize=figsize)
    fig.set_dpi = resolution
    # Labels
    ax.set_xlabel(xlabel, fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12, rotation=90, ha='right')
    ax.set_title(title)
    # Ticks
    ax.minorticks_on()
    ax.tick_params(which='both', axis='both', direction='in')
    return fig, ax

# Handle customization files
if args.custom is not None:
    if not file_exists(args.custom, 'py'):
        with open(args.custom + '.py', 'w') as custom:
            custom.write(custom_default)
    from importlib import import_module
    custom = import_module(args.custom)
else:
    from types import SimpleNamespace
    def get_data(data):
        xs = data[1:, 0]
        ys = data[1:, 1]
        arrays = [xs, ys]
        return arrays

    def modify(fig, ax):
        return fig, ax
    
    custom = SimpleNamespace(modify = modify, get_data = get_data, kwargs = lambda arrays, fig, ax: {})


# If the output format is pgf, we use pdflatex
if args.extension == 'pgf':
    import matplotlib
    matplotlib.use("pgf")
    matplotlib.rcParams.update({
        "pgf.texsystem": "pdflatex",
        'font.family': 'serif',
        'text.usetex': True,
        'pgf.rcfonts': False,
    })
    extension = 'pgf'
else:
    extension = args.extension

# ==============
# BEAUTIFUL PLOT
# ==============

if __name__ == '__main__':
    # fig, ax
    fig, ax = generateplot(figsize_inches)
    # Modify plot
    fig, ax = custom.modify(fig, ax)
    # Get data
    arrays = custom.get_data(data)

    # Plot
    for i in range(len(arrays)-1):
        if scatter:
            ax.scatter(arrays[0], arrays[i+1], marker="o", s=7, zorder=3, **custom.kwargs(arrays, fig, ax))
        else:
            ax.plot(arrays[0], arrays[i+1], **custom.kwargs(arrays, fig, ax, i))

    # Save
    savefigure(fig, file_name, extension)
