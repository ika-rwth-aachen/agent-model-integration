import json
import pathlib
import matplotlib.pyplot as plt
import argparse
import os

def plotVehicleState(args):

    data = json.load(args.file)

    t = []
    s = []
    v = []
    a = []
    d_psi = []
    kappa = []
    delta = []

    # get data
    for d in data:
        t.append(d["time"])
        s.append(d["vehicle_state"]["s"])
        v.append(d["vehicle_state"]["v"])
        a.append(d["vehicle_state"]["a"])
        d_psi.append(d["vehicle_state"]["d_psi"])
        kappa.append(d["vehicle_state"]["kappa"])
        delta.append(d["vehicle_state"]["delta"])
    
    # plot 
    fig, axs = plt.subplots(3,2)    
    
    axs[0, 0].plot(t, s)
    axs[0, 0].set_title('s values')
    axs[0, 0].set(xlabel='t', ylabel='s')
    
    axs[1, 0].plot(t, v)
    axs[1, 0].set_title('v values')
    axs[1, 0].set(xlabel='t', ylabel='v')
    
    axs[2, 0].plot(t, a)
    axs[2, 0].set_title('a values')
    axs[2, 0].set(xlabel='t', ylabel='a')
    
    axs[0, 1].plot(t, d_psi)
    axs[0, 1].set_title('d_psi values')
    axs[0, 1].set(xlabel='t', ylabel='d_psi')

    
    axs[1, 1].plot(t, kappa)
    axs[1, 1].set_title('kappa values')
    axs[1, 1].set(xlabel='t', ylabel='kappa')
    
    axs[2, 1].plot(t, delta)
    axs[2, 1].set_title('delta values')
    axs[2, 1].set(xlabel='t', ylabel='delta')

    plt.suptitle('Vehicle States')

    plt.show()

    if args.save:
        if not args.output.exists():
            os.mkdir(args.output)

        name = 'vehicle_states.png'
        file_name = pathlib.PurePath(args.output, name)
        plt.savefig(file_name, format='png', dpi=1200)

if __name__ == "__main__":
        
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', type=argparse.FileType('r'), help="vehicle_<xy>.json input file generated from ikaDriver")
    parser.add_argument('-s', '--save', action='store_true', help="if figure is saved")
    parser.add_argument('-o', '--output', default=pathlib.Path("debug"), type=pathlib.Path, help="output path")

    args = parser.parse_args()

    plotVehicleState(args)
