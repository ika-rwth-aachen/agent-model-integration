import json
import pathlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import sqrt
import argparse
import os
import time

# paper configuration
import matplotlib as mpl
mpl.use('pgf')
default_width = 3.4 # 3.4 (2 columns)   2.1 (3 columns)
default_ratio = (sqrt(5.0) - 1.0) / 2.0 # golden mean
mpl.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
    "font.serif": [],
    "font.size": 8,
    "font.sans-serif": [],
    "font.monospace": [],
    "figure.figsize": [default_width, default_width * default_ratio],
    "pgf.preamble": [
       # put LaTeX preamble declarations here
       r"\usepackage{graphicx}",
       #r"\usepackage[utf8x]{inputenc}",
       #r"\usepackage[T1]{fontenc}",
       # macros defined here will be available in plots, e.g.:
       # You can use dummy implementations, since you LaTeX document
       # will render these properly, anyway.
   ],
})

# parameters
# colors of the set_level project
red = '#c81414'
dark_blue = '#0a364d'
grey = '#575757'
black = '#000000'
dark_red = '#8c0e0e'
light_blue = '#0a6799'

# figure sizes
cm = 1/2.54  # conversion of inches to cm

figure_size_one = [default_width, default_width * default_ratio]
figure_size_multi = (45*cm, 30*cm)

# set dpi
set_dpi = 300 # print: 300

# location of legend in plot (lower/upper; left/right; best)
legend_location = 'upper left'


# defines the margin to the limit in the plots
def margin(value, divisor = 3.0):
    max_gap = abs(np.max(value) - np.mean(value))
    min_gap = abs(np.mean(value) - np.min(value))
    return max(max_gap, min_gap) / divisor

# helper function for plotting two values. possible to change both variable terms to the desired names 
def helperTwoVal(axs1, axs2, x_70, x_100, x_130, y1_70, y1_100, y1_130, y2_70, y2_100, y2_130, unit1, unit2, label1, label2, name_1 = True, name_2 = True, color_1 = black, color_2 = black):

    axs2.plot(x_70, y2_70, light_blue, linewidth=0.3, linestyle='dashed')
    axs2.plot(x_100, y2_100, red, linewidth=0.3, linestyle='dashed')
    axs2.plot(x_130, y2_130, dark_blue, linewidth=0.3, linestyle='dashed')

    axs1.plot(x_70, y1_70, light_blue, label='$r=70m$')
    axs1.plot(x_100, y1_100, red, label='$r=100m$')
    axs1.plot(x_130, y1_130, dark_blue,label='$r=130m$')
    axs1.legend()

    axs1.tick_params(axis = 'y', which = 'both', right = False, colors = color_1)
    axs2.tick_params(axis = 'y', which = 'both', right = True, labelright = True, left = False, labelleft = False, colors = black)
    plt.ticklabel_format(style='sci', axis='y', scilimits=(-2,2))
    
    axs1.yaxis.set_major_locator(plt.MaxNLocator(5))
    axs2.yaxis.set_major_locator(plt.MaxNLocator(5)) 
    
    axs1.set_xlim(x_70[0], x_70[-1])
    axs1.set_ylim(min(y1_70) - margin(y1_70), max(y1_70) + margin(y1_70))
    axs2.set_ylim(min(y2_70) - 0.5*margin(y2_70), max(y2_70) + 0.5*margin(y2_70))

    axs2.spines['left'].set_color(color_1)
    axs2.spines['right'].set_color(color_2)

    axs1.set_xlabel(f"distance [{r'$m$'}]")
    axs1.set_ylabel(f'{label1} [{unit1}]', color = color_1)
    axs2.set_ylabel(f'{label2} [{unit2}]', color = color_2)
    if name_1 != True:
        axs1.set_ylabel(name_1, color = color_1)
    if name_2 != True:
        axs2.set_ylabel(name_2, color = color_2)
    return axs1, axs2


def plotSpeed():

    plt.style.use(os.path.join(pathlib.PurePath(), "style_single.mplstyle"))

    f1 = open("debug/r70.json", "r")
    f2 = open("debug/r100.json", "r")
    f3 = open("debug/r130.json", "r")

    list_var = ['t', 'follow_distance', 'follow_velocity', 'stop_ds','stop_ds_max', 'velocity_local', 'velocity_prediction',
    'subconscious_a', 'subconscious_d_psi', 'subconscious_kappa',
    'ego_d', 'ego_v', 'ego_a', 'ego_s', 'ego_psi', 'ego_d_psi',
    'vs_s', 'vs_v', 'vs_a', 'vs_d_psi', 'vs_kappa', 'vs_delta',
    'THW', 'hor_kappa']
    
    ####################### get all the data from f1 ###########################
    dict_var_f1 = {var: [] for var in list_var}
    data = json.load(f1)
    for d in data:
        dict_var_f1['t'].append(round(d["time"], 3))

        # driver state
        # conscious
        dict_var_f1['follow_distance'].append(d["driver_state"]["conscious"]["follow"]["distance_0"])
        dict_var_f1['follow_velocity'].append(d["driver_state"]["conscious"]["follow"]["velocity_0"])

        dict_var_f1['stop_ds'].append(d["driver_state"]["conscious"]["stop"]["ds"])
        dict_var_f1['stop_ds_max'].append(d["driver_state"]["conscious"]["stop"]["dsMax"])

        dict_var_f1['velocity_local'].append(d["driver_state"]["conscious"]["velocity"]["local"])
        dict_var_f1['velocity_prediction'].append(d["driver_state"]["conscious"]["velocity"]["prediction"])

        # subconscious
        dict_var_f1['subconscious_a'].append(d["driver_state"]["subconscious"]["a"])
        dict_var_f1['subconscious_d_psi'].append(d["driver_state"]["subconscious"]["dPsi"])
        dict_var_f1['subconscious_kappa'].append(d["driver_state"]["subconscious"]["kappa"])


        # ego input
        dict_var_f1['ego_d'].append(d['ego_input']['d'])
        dict_var_f1['ego_v'].append(d['ego_input']['v'])
        dict_var_f1['ego_a'].append(d['ego_input']['a'])
        dict_var_f1['ego_s'].append(d['ego_input']['s'])
        dict_var_f1['ego_psi'].append(d['ego_input']['psi'])
        dict_var_f1['ego_d_psi'].append(d['ego_input']['dPsi'])


        # vehicle state
        dict_var_f1['vs_s'].append(d['vehicle_state']['s'])
        dict_var_f1['vs_v'].append(d['vehicle_state']['v'])
        dict_var_f1['vs_a'].append(d['vehicle_state']['a'])
        dict_var_f1['vs_d_psi'].append(d['vehicle_state']['d_psi'])
        dict_var_f1['vs_kappa'].append(d['vehicle_state']['kappa'])
        dict_var_f1['vs_delta'].append(d['vehicle_state']['delta'])

        dict_var_f1['hor_kappa'].append(d['horizon']['kappa'][0])

    # convert all nones to zeros
    for i in dict_var_f1:
        for j in range(0, len(dict_var_f1[i])):
            if dict_var_f1[i][j] == None:
                dict_var_f1[i][j] = 0

    # slice values to desired s span
    start = 200
    end = 700
    start_idx = np.argmin(np.abs(np.array(dict_var_f1['ego_s'])-start))
    end_idx = np.argmin(np.abs(np.array(dict_var_f1['ego_s'])-end))
    for i in dict_var_f1:
        dict_var_f1[i] = dict_var_f1[i][start_idx:end_idx]

                
    ####################### get all the data from f1 ###########################
    dict_var_f2 = {var: [] for var in list_var}
    data = json.load(f2)
    for d in data:
        dict_var_f2['t'].append(round(d["time"], 3))

        # driver state
        # conscious
        dict_var_f2['follow_distance'].append(d["driver_state"]["conscious"]["follow"]["distance_0"])
        dict_var_f2['follow_velocity'].append(d["driver_state"]["conscious"]["follow"]["velocity_0"])

        dict_var_f2['stop_ds'].append(d["driver_state"]["conscious"]["stop"]["ds"])
        dict_var_f2['stop_ds_max'].append(d["driver_state"]["conscious"]["stop"]["dsMax"])

        dict_var_f2['velocity_local'].append(d["driver_state"]["conscious"]["velocity"]["local"])
        dict_var_f2['velocity_prediction'].append(d["driver_state"]["conscious"]["velocity"]["prediction"])

        # subconscious
        dict_var_f2['subconscious_a'].append(d["driver_state"]["subconscious"]["a"])
        dict_var_f2['subconscious_d_psi'].append(d["driver_state"]["subconscious"]["dPsi"])
        dict_var_f2['subconscious_kappa'].append(d["driver_state"]["subconscious"]["kappa"])


        # ego input
        dict_var_f2['ego_d'].append(d['ego_input']['d'])
        dict_var_f2['ego_v'].append(d['ego_input']['v'])
        dict_var_f2['ego_a'].append(d['ego_input']['a'])
        dict_var_f2['ego_s'].append(d['ego_input']['s'])
        dict_var_f2['ego_psi'].append(d['ego_input']['psi'])
        dict_var_f2['ego_d_psi'].append(d['ego_input']['dPsi'])


        # vehicle state
        dict_var_f2['vs_s'].append(d['vehicle_state']['s'])
        dict_var_f2['vs_v'].append(d['vehicle_state']['v'])
        dict_var_f2['vs_a'].append(d['vehicle_state']['a'])
        dict_var_f2['vs_d_psi'].append(d['vehicle_state']['d_psi'])
        dict_var_f2['vs_kappa'].append(d['vehicle_state']['kappa'])
        dict_var_f2['vs_delta'].append(d['vehicle_state']['delta'])

        dict_var_f2['hor_kappa'].append(d['horizon']['kappa'][0])

    # convert all nones to zeros
    for i in dict_var_f2:
        for j in range(0, len(dict_var_f2[i])):
            if dict_var_f2[i][j] == None:
                dict_var_f2[i][j] = 0

    # slice values to desired s span
    start = 200
    end = 700
    start_idx = np.argmin(np.abs(np.array(dict_var_f2['ego_s'])-start))
    end_idx = np.argmin(np.abs(np.array(dict_var_f2['ego_s'])-end))
    for i in dict_var_f2:
        dict_var_f2[i] = dict_var_f2[i][start_idx:end_idx]

                
    ####################### get all the data from f1 ###########################
    dict_var_f3 = {var: [] for var in list_var}
    data = json.load(f3)
    for d in data:
        dict_var_f3['t'].append(round(d["time"], 3))

        # driver state
        # conscious
        dict_var_f3['follow_distance'].append(d["driver_state"]["conscious"]["follow"]["distance_0"])
        dict_var_f3['follow_velocity'].append(d["driver_state"]["conscious"]["follow"]["velocity_0"])

        dict_var_f3['stop_ds'].append(d["driver_state"]["conscious"]["stop"]["ds"])
        dict_var_f3['stop_ds_max'].append(d["driver_state"]["conscious"]["stop"]["dsMax"])

        dict_var_f3['velocity_local'].append(d["driver_state"]["conscious"]["velocity"]["local"])
        dict_var_f3['velocity_prediction'].append(d["driver_state"]["conscious"]["velocity"]["prediction"])

        # subconscious
        dict_var_f3['subconscious_a'].append(d["driver_state"]["subconscious"]["a"])
        dict_var_f3['subconscious_d_psi'].append(d["driver_state"]["subconscious"]["dPsi"])
        dict_var_f3['subconscious_kappa'].append(d["driver_state"]["subconscious"]["kappa"])


        # ego input
        dict_var_f3['ego_d'].append(d['ego_input']['d'])
        dict_var_f3['ego_v'].append(d['ego_input']['v'])
        dict_var_f3['ego_a'].append(d['ego_input']['a'])
        dict_var_f3['ego_s'].append(d['ego_input']['s'])
        dict_var_f3['ego_psi'].append(d['ego_input']['psi'])
        dict_var_f3['ego_d_psi'].append(d['ego_input']['dPsi'])


        # vehicle state
        dict_var_f3['vs_s'].append(d['vehicle_state']['s'])
        dict_var_f3['vs_v'].append(d['vehicle_state']['v'])
        dict_var_f3['vs_a'].append(d['vehicle_state']['a'])
        dict_var_f3['vs_d_psi'].append(d['vehicle_state']['d_psi'])
        dict_var_f3['vs_kappa'].append(d['vehicle_state']['kappa'])
        dict_var_f3['vs_delta'].append(d['vehicle_state']['delta'])

        dict_var_f3['hor_kappa'].append(d['horizon']['kappa'][0])

    # convert all nones to zeros
    for i in dict_var_f3:
        for j in range(0, len(dict_var_f3[i])):
            if dict_var_f3[i][j] == None:
                dict_var_f3[i][j] = 0

    # slice values to desired s span
    start = 200
    end = 700
    start_idx = np.argmin(np.abs(np.array(dict_var_f3['ego_s'])-start))
    end_idx = np.argmin(np.abs(np.array(dict_var_f3['ego_s'])-end))
    for i in dict_var_f3:
        dict_var_f3[i] = dict_var_f3[i][start_idx:end_idx]
    
    ################### plotting two values over s #############################
    fig, axs1 = plt.subplots(figsize = figure_size_one, tight_layout = True)
    axs2 = axs1.twinx()

    unit1 = r'${m}/{s}$'
    unit2 = r'$1/{m}$'

    label1 = 'v'
    label2 = r'$\kappa$'

    x_70 = np.array(dict_var_f1['ego_s'])
    x_100 = np.array(dict_var_f2['ego_s'])
    x_130 = np.array(dict_var_f3['ego_s'])
    y1_70 = np.array(dict_var_f1['ego_v'])
    y1_100 = np.array(dict_var_f2['ego_v'])
    y1_130 = np.array(dict_var_f3['ego_v'])
    y2_70 = np.array(dict_var_f1['hor_kappa'])
    y2_100 = np.array(dict_var_f2['hor_kappa'])
    y2_130 = np.array(dict_var_f3['hor_kappa'])

    axs1, axs2 = helperTwoVal(axs1, axs2, x_70, x_100, x_130, y1_70, y1_100, y1_130, y2_70, y2_100, y2_130, unit1, unit2, label1, label2)

    name = 'speed_comparison'

    format_plot = 'pgf'
    file_name = pathlib.PurePath('debug', name)
    plt.savefig(f'{file_name}.{format_plot}', format=format_plot, dpi=set_dpi)

if __name__ == "__main__":
        
    parser = argparse.ArgumentParser()
    start = time.time()
    plotSpeed()
    end = time.time()

print(f'Execution time: {end-start}')