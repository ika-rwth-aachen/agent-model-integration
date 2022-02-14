import json
import pathlib
import matplotlib.pyplot as plt
import argparse
import os

def plotHorizon(args):

    data = json.load(args.file)

    x = []
    y = []
    ds = []
    psi = []
    kappa = []

    # get data
    for d in data:
        t = d["time"]

        if t == args.time:
            x = d["horizon"]["x"]
            y = d["horizon"]["x"]
            ds = d["horizon"]["ds"]
            psi = d["horizon"]["psi"]
            kappa = d["horizon"]["kappa"]
            break
    
    # plot 
    if len(x) > 0:
        fig, axs = plt.subplots(2,2)    
        
        axs[0, 0].plot(x, y, "x-")
        axs[0, 0].axis('equal')
        axs[0, 0].set_title('x-y values')
        axs[0, 0].set(xlabel='x', ylabel='y')

        axs[0, 1].plot(ds)
        axs[0, 1].set_title('ds values')
        axs[0, 1].set(ylabel='ds')

        axs[1, 0].plot(psi)
        axs[1, 0].set_title('psi values')
        axs[1, 0].set(ylabel='psi')

        axs[1, 1].plot(kappa)
        axs[1, 1].set_title('kappa values')
        axs[1, 1].set(ylabel='kappa')
        
        plt.suptitle('Horizon at t=' + str(args.time))

        #mng = plt.get_current_fig_manager()
        #mng.frame.Maximize(True)

        plt.show()

        if args.save:
            if not args.output.exists():
                os.mkdir(args.output)

            name = 'horizon_t=' + str(args.time) + '.eps'
            file_name = pathlib.PurePath(args.output, name)
            plt.savefig(file_name, format='eps', dpi=1200)

if __name__ == "__main__":
        
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', type=argparse.FileType('r'), help="SensorView.json input file")
    parser.add_argument('-s', '--save', action='store_true', help="if figure is saved")
    parser.add_argument('-o', '--output', default=pathlib.Path("../../debug"), type=pathlib.Path, help="output path")
    parser.add_argument('-t', '--time', default=0.0, type=float, help="relevant time")

    args = parser.parse_args()

    plotHorizon(args)
