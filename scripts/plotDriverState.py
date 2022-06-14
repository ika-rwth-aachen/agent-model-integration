import json
import pathlib
import matplotlib.pyplot as plt
import argparse
import os

def plotDriverState(args):

    data = json.load(args.file)

    t = []
    follow_distance = []
    follow_velocity = []
    stop_ds = []
    stop_ds_max = []
    velocity_local = []
    velocity_prediction = []
    subconscious_a = []
    subconscious_d_psi = []
    subconscious_kappa = []

    # get data
    for d in data:
        t.append(d["time"])

        follow_distance.append(d["driver_state"]["conscious"]["follow"]["distance"])
        follow_velocity.append(d["driver_state"]["conscious"]["follow"]["velocity"])
        
        velocity_local.append(d["driver_state"]["conscious"]["velocity"]["local"])
        velocity_prediction.append(d["driver_state"]["conscious"]["velocity"]["prediction"])
       
        stop_ds.append(d["driver_state"]["conscious"]["stop"]["ds"])
        stop_ds_max.append(d["driver_state"]["conscious"]["stop"]["dsMax"])
       
        subconscious_d_psi.append(d["driver_state"]["subconscious"]["dPsi"])
        subconscious_kappa.append(d["driver_state"]["subconscious"]["kappa"])
        subconscious_a.append(d["driver_state"]["subconscious"]["a"])
    
    # plot 
    fig, axs = plt.subplots(3,3)    
    
    axs[0, 0].plot(t, subconscious_a)
    axs[0, 0].set_title('subconscious_a values')
    axs[0, 0].set(xlabel='t', ylabel='subconscious_a')
    
    axs[0, 1].plot(t, subconscious_kappa)
    axs[0, 1].set_title('subconscious_kappa values')
    axs[0, 1].set(xlabel='t', ylabel='subconscious_kappa')
    
    axs[0, 2].plot(t, subconscious_d_psi)
    axs[0, 2].set_title('subconscious_d_psi values')
    axs[0, 2].set(xlabel='t', ylabel='subconscious_d_psi')
    

    axs[1, 0].plot(t, follow_distance)
    axs[1, 0].set_title('follow_distance values')
    axs[1, 0].set(xlabel='t', ylabel='follow_distance')
    
    axs[2, 0].plot(t, follow_velocity)
    axs[2, 0].set_title('follow_velocity values')
    axs[2, 0].set(xlabel='t', ylabel='follow_velocity')
    

    axs[1, 1].plot(t, velocity_local)
    axs[1, 1].set_title('velocity_local values')
    axs[1, 1].set(xlabel='t', ylabel='velocity_local')
    
    axs[2, 1].plot(t, velocity_prediction)
    axs[2, 1].set_title('velocity_prediction values')
    axs[2, 1].set(xlabel='t', ylabel='velocity_prediction')
    

    axs[1, 2].plot(t, stop_ds)
    axs[1, 2].set_title('stop_ds values')
    axs[1, 2].set(xlabel='t', ylabel='stop_ds')
    
    axs[2, 2].plot(t, stop_ds_max)
    axs[2, 2].set_title('stop_ds_max values')
    axs[2, 2].set(xlabel='t', ylabel='stop_ds_max')

    plt.suptitle('Driver States')

    plt.show()

    if args.save:
        if not args.output.exists():
            os.mkdir(args.output)

        name = 'driver_states.eps'
        file_name = pathlib.PurePath(args.output, name)
        plt.savefig(file_name, format='eps', dpi=1200)

if __name__ == "__main__":
        
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', type=argparse.FileType('r'), help="vehicle_<xy>.json input file generated from ikaDriver")
    parser.add_argument('-s', '--save', action='store_true', help="if figure is saved")
    parser.add_argument('-o', '--output', default=pathlib.Path("debug"), type=pathlib.Path, help="output path")

    args = parser.parse_args()

    plotDriverState(args)
