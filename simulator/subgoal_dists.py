from scipy import stats
import numpy as np

# FILL THESE IN OR LOAD FROM PICKLE
sg_locs = np.array([[ 6.68740728, 0., 4.48401146, 1.2 ],
                    [ 4.8347579 , 0., 4.48401146, 1.2 ],
                    [ 4.8347579 , 5.53092909,  9.84963903,  1.2       ],
                    [  8.76074058, 5.53092909,  10.50560411,   1.2       ],
                    [  8.03953139, 3.3289028 ,  10.50560411,   0.3       ],
                    [ 7.51114802, 3.3289028, 2.49584783, 0.3]])

sg_scales = np.repeat(np.array([[3, 3, 2, 0.2]]), 6, axis=0)

sg_dists = [stats.norm(loc=means, scale=scale) for (means, scale) in zip(sg_locs, sg_scales)]
