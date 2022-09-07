# commands to get the plots used in the set_level ASE presentation
python3 plotState.py -f ~/set_level/ika-driver-setlevel/debug/vehicle_16.json -p subconscious_a -p2 subconscious_kappa -a -t 23.0-40.0 -s
python3 plotState.py -f ~/set_level/ika-driver-setlevel/debug/vehicle_18.json -p vs_v -p2 subconscious_a -a -t 0.0-13.5 -s
python3 plotState.py -f ~/set_level/ika-driver-setlevel/debug/vehicle_1.json -p ego_d -p2 ego_psi -a -t 7.0-22.0 -s
python3 plotState.py -f ~/set_level/ika-driver-setlevel/debug/vehicle_4.json -p subconscious_a -p2 THW -a -t 0.0-15.0 -s