# Using the deviations from sgs_1101.py
# combined with demo'd locations in the workspace
# demos analyzed in jupyter/IFAC_ext/sg-locations.ipynb

# See sg_data_1101.csv for source data and sg_data_1101_calcs.xlsx for calcs on variance

sg_model = [{'subgoal': 1,
             'npt': [1, 2, 1.5, 0.25],
             'onpt': [],
             'it': [3, 0.5],
             'subgoal_pos': [9.74071429, 5.07757037, 4.10446154, 1.29099823]},
            {'subgoal': 2,
             'npt': [1, 1.5, 2, 0.25],
             'onpt': [],
             'it': [0, -0.5],
             'subgoal_pos': [7.36845161, 3.56592593, 4.13942308, 1.29688872]},
            {'subgoal': 3,
             'npt': [1, 2.5, 1, 0.25],
             'onpt': [],
             'it': [1, 0.5],
             'subgoal_pos': [7.36845161, 7.0824, 9.76439286, 1.32241416]},
            {'subgoal': 4,
             'npt': [1, 2.5, 1, 0.25],
             'onpt': [],
             'it': [0, 0.5],
             'subgoal_pos': [10.202,    9.778,  9.559,  1.245]},
            {'subgoal': 5,
             'npt': [1, 1.8, 1, 0.1],
             'onpt': [],
             'it': [3, -0.5],
             'subgoal_pos': [10.239,    9.364,  9.747,  0.011]},
            {'subgoal': 6,
             'npt': [1, 1.3, 1.2, 0.1],
             'onpt': [],
             'it': [2, -0.5],
             'subgoal_pos': [10.239,    9.190,  2.598,  0.013]}]
