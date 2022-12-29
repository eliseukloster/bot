
# ============
# EXTRACT DATA
# ============

def get_data(data):
    ys0 = data
    #ys1 = data[:,1]
    xs = [i for i in range(len(data))]
    arrays = [xs, ys0]
    return arrays

# ==============
# CUSTOMIZE PLOT
# ==============

def modify(fig, ax):
    return fig, ax

# ========================
# CUSTOMIZE plot arguments
# ========================

def kwargs(arrays, fig, ax, i=0):
    kwargs = {
        #'c': '#000000',
        'linewidth': 0.75,
    }
    return kwargs
