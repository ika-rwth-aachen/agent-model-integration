import json
import pathlib
import matplotlib.pyplot as plt
import argparse
import os

def plotEgoInput(args):

    data = json.load(args.file)

    t = []
    d = []
    v = []
    a = []
    s = []
    #d_psi = []
    #kappa = []
    #delta = []

    # get data
    for dd in data:
        t.append(dd["time"])
        d.append(dd["ego_input"]["d"])
        v.append(dd["ego_input"]["v"])
        a.append(dd["ego_input"]["a"])
        s.append(dd["ego_input"]["s"])
    
    # plot 
    fig, axs = plt.subplots(2,2)    
    
    axs[0, 0].plot(t, s)
    axs[0, 0].set_title('distance')
    axs[0, 0].set(xlabel='t', ylabel='s')
    
    axs[1, 0].plot(t, d)
    axs[1, 0].set_title('offset')
    axs[1, 0].set(xlabel='t', ylabel='d')
    
    axs[0, 1].plot(t, a)
    axs[0, 1].set_title('acc.')
    axs[0, 1].set(xlabel='t', ylabel='a')
    
    axs[1, 1].plot(t, v)
    axs[1, 1].set_title('velocity')
    axs[1, 1].set(xlabel='t', ylabel='v')

    plt.suptitle('Ego Input')

    plt.show()

    if args.save:
        if not args.output.exists():
            os.mkdir(args.output)

        name = 'ego_input.eps'
        file_name = pathlib.PurePath(args.output, name)
        plt.savefig(file_name, format='eps', dpi=1200)

if __name__ == "__main__":
        
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', type=argparse.FileType('r'), help="vehicle_<xy>.json input file generated from ikaDriver")
    parser.add_argument('-s', '--save', action='store_true', help="if figure is saved")
    parser.add_argument('-o', '--output', default=pathlib.Path("debug"), type=pathlib.Path, help="output path")

    args = parser.parse_args()

    plotEgoInput(args)
