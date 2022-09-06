import json
import pathlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse
import os
import time

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
figure_size_one = (7*cm, 4.1*cm)
figure_size_multi = (45*cm, 30*cm)

# set dpi
set_dpi = 300 # 300: print

# location of legend in plot (lower/upper; left/right; best)
legend_location = 'upper left'


# defines the margin to the limit in the plots
def margin(value, divisor = 3.0):
    max_gap = abs(np.max(value) - np.mean(value))
    min_gap = abs(np.mean(value) - np.min(value))
    return max(max_gap, min_gap) / divisor

# helper function for plotting one value
def helperOneVal(axs, x, y, p, unit, a = False, name = True, color = light_blue):
    if not a:
        axs.plot(x, y, color)

    axs.set_xlim(x[0], x[-1])
    axs.set_ylim(min(y) - margin(y), max(y) + margin(y))

    axs.set_title(f'{p} values')
    axs.set(xlabel=f"time [{r'$s$'}]", ylabel=f'{p} [{unit}]')
    if not name:
         axs.set_ylabel(name, color = color)
    return axs

# helper function for plotting two values. possible to change both variable terms to the desired names 
def helperTwoVal(axs1, axs2, x, y1, y2, p, p2, unit1, unit2, a = False, name_1 = True, name_2 = True, color_1 = light_blue, color_2 = dark_red):

    if not a:
        axs1.plot(x, y1, color_1)
        axs2.plot(x, y2, color_2)

    axs1.tick_params(axis = 'y', which = 'both', right = False, colors = color_1)
    axs2.tick_params(axis = 'y', which = 'both', right = True, labelright = True, left = False, labelleft = False, colors = color_2)
    plt.ticklabel_format(style='sci', axis='y', scilimits=(-2,2))
    
    axs1.yaxis.set_major_locator(plt.MaxNLocator(5))
    axs2.yaxis.set_major_locator(plt.MaxNLocator(5)) 
    
    axs1.set_xlim(x[0], x[-1])
    axs1.set_ylim(min(y1) - margin(y1), max(y1) + margin(y1))
    axs2.set_ylim(min(y2) - margin(y2), max(y2) + margin(y2))

    axs2.spines['left'].set_color(color_1)
    axs2.spines['right'].set_color(color_2)

    axs1.set_xlabel(f"time [{r'$s$'}]")
    axs1.set_ylabel(f'{p} [{unit1}]', color = color_1)
    axs2.set_ylabel(f'{p2} [{unit2}]', color = color_2)
    if not name_1:
        axs1.set_ylabel(name_1, color = color_1)
    if not name_2:
        axs2.set_ylabel(name_2, color = color_2)
    return axs1, axs2


def plotState(args):

    p = args.plot
    p2 = args.plot2

    if args.plot in {'driver_state', 'ego_input', 'vehicle_state'}:
        plt.style.use(os.path.join(pathlib.PurePath(), "style_multi.mplstyle"))
    else:
        plt.style.use(os.path.join(pathlib.PurePath(), "style_single.mplstyle"))

    data = json.load(args.file)

    list_var = ['t', 'follow_distance', 'follow_velocity', 'stop_ds','stop_ds_max', 'velocity_local', 'velocity_prediction',
    'subconscious_a', 'subconscious_d_psi', 'subconscious_kappa',
    'ego_d', 'ego_v', 'ego_a', 'ego_s', 'ego_psi', 'ego_d_psi',
    'vs_s', 'vs_v', 'vs_a', 'vs_d_psi', 'vs_kappa', 'vs_delta',
    'THW']

    list_var_unit = [r'$s$', r'$m$', r'$\frac{m}{s}$', r'$m$', r'$m$', r'$\frac{m}{s}$', r'$\frac{m}{s}$', r'$\frac{m}{s^2}$', r'$rad$', r'$rad$', 
    r'$m$', r'$\frac{m}{s}$', r'$\frac{m}{s^2}$', r'$m$', r'$rad$', r'$rad$',
    r'$m$', r'$\frac{m}{s}$', r'$\frac{m}{s^2}$', r'$rad$', r'$rad$', r'$m$',
    r'$s$']

    dict_var = {var: [] for var in list_var}

    # get all the data
    for d in data:
        dict_var['t'].append(d["time"])

        # driver state
        # conscious
        dict_var['follow_distance'].append(d["driver_state"]["conscious"]["follow"]["distance_0"])
        dict_var['follow_velocity'].append(d["driver_state"]["conscious"]["follow"]["velocity_0"])

        dict_var['stop_ds'].append(d["driver_state"]["conscious"]["stop"]["ds"])
        dict_var['stop_ds_max'].append(d["driver_state"]["conscious"]["stop"]["dsMax"])

        dict_var['velocity_local'].append(d["driver_state"]["conscious"]["velocity"]["local"])
        dict_var['velocity_prediction'].append(d["driver_state"]["conscious"]["velocity"]["prediction"])

        # subconscious
        dict_var['subconscious_a'].append(d["driver_state"]["subconscious"]["a"])
        dict_var['subconscious_d_psi'].append(d["driver_state"]["subconscious"]["dPsi"])
        dict_var['subconscious_kappa'].append(d["driver_state"]["subconscious"]["kappa"])


        # ego input
        dict_var['ego_d'].append(d['ego_input']['d'])
        dict_var['ego_v'].append(d['ego_input']['v'])
        dict_var['ego_a'].append(d['ego_input']['a'])
        dict_var['ego_s'].append(d['ego_input']['s'])
        dict_var['ego_psi'].append(d['ego_input']['psi'])
        dict_var['ego_d_psi'].append(d['ego_input']['dPsi'])


        # vehicle state
        dict_var['vs_s'].append(d['vehicle_state']['s'])
        dict_var['vs_v'].append(d['vehicle_state']['v'])
        dict_var['vs_a'].append(d['vehicle_state']['a'])
        dict_var['vs_d_psi'].append(d['vehicle_state']['d_psi'])
        dict_var['vs_kappa'].append(d['vehicle_state']['kappa'])
        dict_var['vs_delta'].append(d['vehicle_state']['delta'])

    # convert all nones to zeros
    for i in dict_var:
        for j in range(0, len(dict_var[i])):
            if dict_var[i][j] == None:
                dict_var[i][j] = 0

    # slice values to desired time span
    if args.time != 'all':
        start = args.time.split('-')[0]
        end = args.time.split('-')[1]
        start_idx = dict_var['t'].index(float(start))
        end_idx = dict_var['t'].index(float(end))
        for i in dict_var:
            if i == 't':
                continue
            else:
                dict_var[i] = dict_var[i][start_idx:end_idx]
        dict_var['t'] = [x / 10.0 for x in range(0, int((float(end)-float(start))*10), 1)] # start time always from 0

    # add time headway to the dict, definition: relative distance to leading vehicle divided by the driving speed of driver’s own vehicle
    np_vs_v = np.array(dict_var['vs_v'])
    dict_var['THW'] = [i / j if j != 0 else i / (j+np.percentile(np_vs_v, 20)) for i,j in zip(dict_var['follow_distance'], dict_var['vs_v'])]


    if args.plot == 'driver_state':
        fig, axs = plt.subplots(3,3, figsize = figure_size_multi)
        var_list = list(dict_var.values())
        k = 1 # driver state entries: 1 - 9
        for i in range(0, 3):
            for j in range (0, 3):
                    axs[i, j].plot(dict_var['t'], var_list[k])
                    axs[i, j].set_title(f'{list(dict_var.keys())[k]} values')
                    axs[i, j].set(xlabel=r'time [$s$]', ylabel=f'{list(dict_var.keys())[k]} [{list_var_unit[k]}]')
                    k += 1
        plt.suptitle('Driver States')
        plt.tight_layout()
        plt.show()
        name = 'driver_state'
    

    if args.plot == 'ego_input':
        fig, axs = plt.subplots(2,2, figsize = figure_size_multi)
        var_list = list(dict_var.values())
        k = 10 # ego input entries: 10 - 13
        for i in range(0, 2):
            for j in range(0, 2):
                axs[i, j].plot(dict_var['t'], var_list[k])
                axs[i, j].set_title(f'{list(dict_var.keys())[k]} values')
                axs[i, j].set(xlabel=r'time [$s$]', ylabel=f'{list(dict_var.keys())[k]} [{list_var_unit[k]}]')
                k += 1
        plt.suptitle('Ego Inputs')
        plt.tight_layout()
        plt.show()
        name = 'ego_input'


    if args.plot == 'vehicle_state':
        fig, axs = plt.subplots(3, 2, figsize = figure_size_multi)
        var_list = list(dict_var.values())
        k = 14 # vehicle state entries: 14 - 19
        for i in range(0, 3):
            for j in range(0, 2):
                axs[i, j].plot(dict_var['t'], var_list[k])
                axs[i, j].set_title(f'{list(dict_var.keys())[k]} values')
                axs[i, j].set(xlabel=r'time [$s$]', ylabel=f'{list(dict_var.keys())[k]} [{list_var_unit[k]}]')
                k += 1
        plt.suptitle('Vehicle States')
        plt.tight_layout()
        plt.show()
        name = 'vehicle_state'


    # for plotting only one value over t
    if p in list(dict_var.keys()) and p2 == 'none' and not args.animation:
        fig, axs = plt.subplots(figsize = figure_size_one, tight_layout = True)

        x = np.array(dict_var['t'])
        y = np.array(dict_var[p])
        unit = list_var_unit[list(dict_var.keys()).index(p)]
        axs = helperOneVal(axs, x, y, p, unit)

        plt.show()
        name = f'{p}'
        
    if p in list(dict_var.keys()) and p2 == 'none' and args.animation:
        fig, axs = plt.subplots(figsize = figure_size_one, tight_layout = True)
        line, = axs.plot([], [], light_blue)

        unit = list_var_unit[list(dict_var.keys()).index(p)]

        x = np.array(dict_var['t'])
        y = np.array(dict_var[p])

        axs = helperOneVal(axs, x, y, p, unit, a = True) # pass here alternative name or color as arguments

        line.set_data([], [])
        def animate_one(i):
            line.set_data(x[:i], y[:i])
            return line,

        anim = animation.FuncAnimation(fig, animate_one, frames = x.size + 1, interval = 50, blit = True) # set speed over interval: 100=normal, 50=double
        plt.show()
        name = f'{p}'
    
    # plotting two values over t
    if p in list(dict_var.keys()) and p2 in list(dict_var.keys()) and not args.animation:
        fig, axs1 = plt.subplots(figsize = figure_size_one, tight_layout = True)
        axs2 = axs1.twinx()

        unit1 = list_var_unit[list(dict_var.keys()).index(p)]
        unit2 = list_var_unit[list(dict_var.keys()).index(p2)]

        x = np.array(dict_var['t'])
        y1 = np.array(dict_var[p])
        y2 = np.array(dict_var[p2])

        axs1, axs2 = helperTwoVal(axs1, axs2, x, y1, y2, p, p2, unit1, unit2)
        name = f'{p}_and_{p2}'
        plt.show()
    
    if p in list(dict_var.keys()) and p2 in list(dict_var.keys()) and args.animation:
        fig, axs1 = plt.subplots(figsize = figure_size_one, tight_layout = True)
        axs2 = axs1.twinx()

        line1, = axs1.plot([], [], light_blue)
        line2, = axs2.plot([], [], dark_red)

        unit1 = list_var_unit[list(dict_var.keys()).index(p)]
        unit2 = list_var_unit[list(dict_var.keys()).index(p2)]

        x = np.array(dict_var['t'])
        y1 = np.array(dict_var[p])
        y2 = np.array(dict_var[p2])

        axs1, axs2 = helperTwoVal(axs1, axs2, x, y1, y2, p, p2, unit1, unit2, a = True) # pass here alternative names or colors as arguments

        line1.set_data([], [])
        line2.set_data([], [])

        def animate_two(k):
            line1.set_data(x[:k], y1[:k])
            line2.set_data(x[:k], y2[:k])
            return line1, line2

        lines = [line1, line2]
        axs1.legend(lines, [f'{p}', f'{p2}'], frameon = True, loc = legend_location)

        # change properties here if tight_layout not preferred
        #plt.subplots_adjust(left=0.19, right=0.77, bottom=0.25, top=0.88, wspace=0.5)

        anim = animation.FuncAnimation(fig, animate_two, frames = x.size + 1, interval = 50, blit = True) # set speed over interval: 100=normal, 50=double
        name = f'{p}_and_{p2}'
        plt.show()


    if args.save and not args.animation:
        if not args.output.exists():
            os.mkdir(args.output)

        format_plot = 'png'
        file_name = pathlib.PurePath(args.output, name)
        plt.savefig(f'{file_name}.{format_plot}', format=format_plot, dpi=set_dpi)

    if args.save and args.animation:
        if not args.output.exists():
            os.mkdir(args.output)

        format_anim = 'mp4'
        file_name = pathlib.PurePath(args.output, name)
        anim.save(f'{file_name}.{format_anim}', dpi=set_dpi)

if __name__ == "__main__":
        
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', type=argparse.FileType('r'), help='vehicle_<xy>.json input file generated from ikaDriver')
    parser.add_argument('-p', '--plot', default='driver_state', help='plot a specific value or all of driver_state, ego_input or vehicle_state')
    parser.add_argument('-p2', '--plot2', default='none', help='adds another value to the plot')
    parser.add_argument('-a', '--animation', action='store_true', help='animates the given plot')
    parser.add_argument('-t', '--time', default='all', help='slices the values to the desired time span, e.g. 10-20')
    parser.add_argument('-s', '--save', action='store_true', help='if figure is saved')
    parser.add_argument('-o', '--output', default=pathlib.Path("debug"), type=pathlib.Path, help='output path')

    args = parser.parse_args()
    start = time.time()
    plotState(args)
    end = time.time()

print(f'Execution time: {end-start}')