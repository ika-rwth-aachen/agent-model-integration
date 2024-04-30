'''
This program plots the ground truth of a serialized osi trace file 

Example usage:
    python3 plotGroundTruthTrace.py -d ../test/traces/SensorView.osi -s -dl -i 2
'''

from OSITrace import OSITrace
from osi3.osi_sensorview_pb2 import SensorView
from osi3.osi_groundtruth_pb2 import GroundTruth
import struct
import lzma
import argparse
import numpy as np
import os
import matplotlib.pyplot as plt

def command_line_arguments():
    """ Define and handle command line interface """

    dir_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    parser = argparse.ArgumentParser(
                        description='Plot the GrountTruth of a serialized osi trace file.',
                        prog='plotGroundTruthTrace plot script')
    parser.add_argument('--data', '-d',
                        help='Path to the file with serialized data.',
                        type=str)
    parser.add_argument('--output', '-o',
                        help='Output name of png file.',
                        default='lanes.png',
                        type=str,
                        required=False)
    parser.add_argument('--save', '-s',
                        help='Flag if a picture of the plot is saved.',
                        action='store_true',
                        required=False)
    parser.add_argument('--driving', '-dl',
                        help='Flag if only driving lanes should be plotted.',
                        action='store_true',
                        required=False)
    parser.add_argument('--index', '-i',
                        help='Index of GT which shall be plotted when multiple SensorView are in trace file.',
                        default=1,
                        type=int,
                        required=False)

    return parser.parse_args()

def main():
    # Handling of command line arguments
    args = command_line_arguments()

    # Initialize the OSI trace class
    trace = OSITrace()
    trace.from_file(path=args.data)

    test = trace.get_messages()
    gt = GroundTruth
    n = args.index
    i = 1
    for sv in test:
        gt = sv.global_ground_truth
        if i == n:
            break
        i = i+1

    ids = []
    centerlines = []
    for lane in gt.lane:
        lane_id = lane.id.value
        #print(lane.classification.type)
        if args.driving:
            if not lane.classification.type == 2:
                continue
        else:
            if lane.classification.type == 4:
                continue
        line = np.empty((0,2), dtype=float)
        for point in lane.classification.centerline:
            line = np.append(line, [[point.x, point.y]], axis=0)
        ids.append(lane_id)
        centerlines.append(line)
        #print(lane.id.value)

    for i in range(len(centerlines)):
        id = str(ids[i])
        line = centerlines[i]

        numPoints = len(line)
        indexMiddlePoint = int(numPoints/2)
        plt.plot(line[:,0], line[:,1], "x-")
        plt.text(line[indexMiddlePoint,0], line[indexMiddlePoint,1],id, va='center', ha='center')

    plt.title('Ground Truth Lanes')
    plt.show()

    if args.save:
        file_name = args.output
        plt.savefig(file_name)   
 
if __name__ == "__main__":
    main()