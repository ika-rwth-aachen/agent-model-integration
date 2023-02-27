# Paper Plots for IROS 2023
# Following
python3 plotState.py -f debug/following.json -p THW -p2 subconscious_a -t 10-80 -s #-a

# Signal
python3 plotState.py -f debug/signal.json -p vs_v -p2 subconscious_a -t 10-30 -s #-a

# Lane Change
python3 plotState.py -f debug/lanechange.json -p ego_d -p2 subconscious_kappa -t 0-10 -s #-a