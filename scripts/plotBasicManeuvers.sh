# commands to get the plots used in the set_level ASE presentation
python3 plotState.py -f ../debug/vehicle_16_16_2023-02-10_13-43-14.json -p subconscious_a -p2 subconscious_kappa -t 23.0-40.0 -s
python3 plotState.py -f ../debug/vehicle_18_18_2023-02-10_13-43-14.json -p vs_v -p2 subconscious_a -t 0.0-13.5 -s
python3 plotState.py -f ../debug/vehicle_1_1_2023-02-15_15-46-37.json -p ego_d -p2 subconscious_kappa -t 7.0-22.0 -s
python3 plotState.py -f ../debug/vehicle_4_4_2023-02-10_13-43-14.json -p THW -p2 subconscious_a -t 0.0-15.0 -s