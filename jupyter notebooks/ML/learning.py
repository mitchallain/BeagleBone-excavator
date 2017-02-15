#! /usr/bin/env python

##########################################################################################
# learning.py
#
# A module for learning from demonstration routines, extracted from jupyter notebooks
#
# NOTE: See jupyter notebooks
#
# Created: February 06, 2017
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import matplotlib.pyplot as plt
import matplotlib
import pandas as pd
import numpy as np
from sklearn.cluster import KMeans
import scipy.stats
from sklearn.preprocessing import normalize
import pdb


def compute_rate(filepath):
    '''Computes velocities using diff and applies smoothing

    Args:
        filepath (str): path to log file with N samples

    Returns:
        rate_sm (np.array): 4 x N array of smoothed rates
    '''

    trial = pd.read_csv(filepath)
    pos = trial[['Time', 'Boom Ms', 'Stick Ms', 'Bucket Ms', 'Swing Ms']].values.transpose()

    # Produce rate array
    rate = np.copy(pos[1:, 1:])

    for i in range(len(rate)):
        rate[i] = (np.divide(np.diff(pos[i+1]), np.diff(pos[0])))

    # Need to cutoff extra data points introduced by smooth function
    # Right now window must be even because laziness
    window_size = 20
    st, sp = (window_size / 2) - 1, -(window_size / 2)

    rate_sm = np.copy(rate)

    for i in range(len(rate)):
        rate_sm[i] = smooth(rate[i], window_size)[st:sp]

    assert (len(rate_sm[0]) == len(pos[0][:-1]))

    return pos, rate_sm


def k_means_action_primitives(rate, threshold=False, eta=None, swap=True):
    ''' Creates kmeans action primitive labeling for rate vector

    Args:
        rate (np.array): N length vector of velocities (from function compute_rate)
        threshold (bool): decides whether to threshold the zero velocity cluster
        eta (float): threshold on relative probability, autocomputes if None
        swap (bool): puts 1 on most negative mean, 2 on middle, 3 on most positive

    Returns:
        sw_labels (np.array): N length vector of swapped clustering labels
    '''

    rate = rate.reshape(-1, 1)
    bm_cluster = KMeans(n_clusters=3, random_state=0).fit(rate)

    labels = np.copy(bm_cluster.labels_)

    means = []
    variance = []
    dists = []

    for label in range(3):
        grouped = [rate[i] for i in range(len(rate)) if bm_cluster.labels_[i] == label]
        means.append(np.mean(grouped))
        variance.append(np.var(grouped))
        dists.append(scipy.stats.norm(np.mean(grouped), np.std(grouped)))

    mean_sorted_indexes = tuple(sorted(range(len(means)), key=lambda k: means[k]))
    dists = [dists[i] for i in mean_sorted_indexes]
    # pdb.set_trace()

    swap_set = (1, 2, 3)

    if threshold:

        if eta is None:
            eta = dists[0].pdf(0.2)

        for i, cluster in enumerate(bm_cluster.labels_):
            # pdb.set_trace()
            if cluster == mean_sorted_indexes[1] and (dists[1].pdf(rate[i]) < eta):
                if dists[0].pdf(rate[i]) > dists[2].pdf(rate[i]):
                    labels[i] = mean_sorted_indexes[0]
                else:
                    labels[i] = mean_sorted_indexes[2]
            else:
                labels[i] = cluster

    # Swap labels
    sw_labels = np.copy(labels)

    if swap:
        for i, label in enumerate(labels):
            sw_labels[i] = swap_set[mean_sorted_indexes.index(label)]

    return sw_labels


def get_action_primitives(rates, threshold, label_set=(1, 2, 3)):
    ''' Basic action primitive scheme, designed to mirror
        k_means_action_primitives() API.

    Args:
        rates (np.array): N length vector of velocities (from function compute_rate)
        threshold (float): zero velocity noise threshold, all larger magnitude velocities are moving actions
        labels (tuple): tuple of labels for negative, zero, and positive velocity, respectively

    Returns:
        labels (np.array): N length vector of clustering labels
    '''
    labels = np.copy(rates)

    for i in range(len(rates)):
        if rates[i] > threshold:
            labels[i] = label_set[2]
        elif rates[i] < - threshold:
            labels[i] = label_set[0]
        else:
            labels[i] = label_set[1]

    return labels


def cluster_plot(time, data, clusters):
    colors = ['red', 'green', 'blue']
    fig = plt.scatter(time, data, c=clusters,
                      cmap=matplotlib.colors.ListedColormap(colors))
#     plt.title('K-Means without Thresholding')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Boom Velocity (mm/s)')

    cbar = plt.colorbar(ticks=[1, 2, 3])
    cbar.ax.set_yticklabels(['1', '2', '3'])  # colorbar
    plt.tight_layout()
    return fig


def cluster_plot_new(time, data, clusters):
    colors = ['red', 'green', 'blue']
    fig = plt.scatter(time, data, c=clusters,
                      cmap=matplotlib.colors.ListedColormap(colors), marker='.')
#     plt.title('K-Means without Thresholding')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Boom Velocity (mm/s)')

    cbar = plt.colorbar(ticks=[1, 2, 3])
    cbar.ax.set_yticklabels(['1', '2', '3'])  # colorbar
    plt.tight_layout()
    return fig


def rational_actor(subgoal, state):
    '''Returns the action primitive tuple corresponding to rational cl controller

    Args:
        subgoal (list): n-length vector corresponding to subgoal location
        state (list): n-length vector for current state

    Returns:
        action_class (np.array): n-length array with action class
            e.g., array([1, -1, 1, -1])
    '''
    action_class = []

    for (sg, st) in zip(subgoal, state):
        action_class.append(np.sign(sg - st))

    return np.array(action_class)


def rational_actor_new(subgoal, state, code=(1, 2, 3), threshold=5):
    '''Returns the action primitive tuple corresponding to rational cl controller

    Args:
        subgoal (list): n-length vector corresponding to subgoal location
        state (list): n-length vector for current state

    Returns:
        action_class (np.array): n-length array with action class
            e.g., array([1, -1, 1, -1])
    '''
    action_class = []

    for (sg, st) in zip(subgoal, state):
        if abs(sg - st) < threshold:
            action_class.append(code[1])
        else:
            action_class.append(code[int(np.sign(sg - st)) + 1])

    return np.array(action_class)


def bnirl_sampling(states, partitions, primitives, verbose=False, debug=False, eta=1):
    '''One Gibbs sampling sweep,
    run in for loop until it converges

    Args:
        states [np.array]: m x n where n is actuators and m is samples
        partitions [list]: m x 1 partition label for each sample
        primitives [np.array]: m x n array describing motion at sample
            e.g., primitives[0] = [1, 0, -1, 1]
                1: positive velocity or input
                0: no velocity,
                -1: negative velocity
        verbose [bool]: gives more info, for debugging
        eta [int]: hyperparameter for CRP

    Returns:
        partitions [list]: updated partitions after Gibbs sweep
        '''
    dim = len(states)

    if debug:
        pdb.set_trace()

    # For each observation
    for i, state in enumerate(states):
        # Start by recomputing subgoal locations from partition means
        sg_means = dict()
        for j in set(partitions):
            sg_mean = []
            for act in states.transpose():
                sg_mean.append(np.mean([act[k] for k in range(dim) if (partitions[k] == j)]))
            sg_means[j] = np.array(sg_mean)

        if verbose:
            print('After observation %i, there are %i partitions with means %s') % (i, len(set(partitions)), str(sg_means))

        part_posterior = []

        # For each existing partition
        for j in set(partitions):  # existing partitions
            crp = partitions.count(j) / float(dim - 1 + eta)
#             pdb.set_trace()
            act_likelihood = np.e**(np.linalg.norm(primitives[i] - rational_actor_new(sg_means[j], state)))
#             act_likelihood = 1
            # Compute partition assignment posterior
            part_posterior.append(crp*act_likelihood)
        # Draw new subgoal uniformly from demo states
        new_sg = states[np.random.randint(0, dim)]
        crp = eta / float(dim - 1 + eta)
        act_likelihood = np.e**(np.linalg.norm(primitives - rational_actor_new(new_sg, state)))
#         act_likelihood = 1
        part_posterior.append(crp*act_likelihood)

        # Normalize the posterior
        part_posterior = np.array(part_posterior) / float(sum(part_posterior))

        if debug:
            pdb.set_trace()

        # Assign partition
        partitions[i] = int(np.random.choice(list(set(partitions)) + [max(set(partitions)) + 1],
                                            1, p=part_posterior))

        # pdb.set_trace()

    return partitions


def bnirl_sampling_2(states, partitions, primitives, subgoals=dict(), verbose=False, debug=False, eta=1):
    '''One Gibbs sampling sweep,
    run in for loop until it converges

    Args:
        states [np.array]: m x n where n is actuators and m is samples
        partitions [list]: m x 1 partition label for each sample
        subgoals [dict]: maps partition number to subgoal location
        primitives [np.array]: m x n array describing motion at sample
            e.g., primitives[0] = [1, 0, -1, 1]
                1: positive velocity or input
                0: no velocity,
                -1: negative velocity
        verbose [bool]: gives more info, for debugging
        eta [int]: hyperparameter for CRP

    Returns:
        partitions [list]: updated partitions after Gibbs sweep
        subgoals [dict]: maps new partition numbers to subgoal location
        '''
    dim = len(states)

    if debug:
        pdb.set_trace()

    # Initialize first partition subgoal to first state
    if not subgoals:
        subgoals[0] = np.array(states[0])

    # For each observation
    for i, state in enumerate(states):
        if verbose:
            print('After observation %i, there are %i partitions at %s') % (i, len(set(partitions)), str(subgoals.replace(',', '\n')))

        part_posterior = []

        # For each existing partition
        for j in set(partitions):  # existing partitions
            crp = partitions.count(j) / float(dim - 1 + eta)
#             pdb.set_trace()
            act_likelihood = np.e**(np.linalg.norm(primitives[i] - rational_actor_new(subgoals[j], state)))
#             act_likelihood = 1
            # Compute partition assignment posterior
            part_posterior.append(crp*act_likelihood)
        # Draw new subgoal uniformly from demo states
        new_sg = states[np.random.randint(0, dim)]
        crp = eta / float(dim - 1 + eta)
        act_likelihood = np.e**(np.linalg.norm(primitives - rational_actor_new(new_sg, state)))
#         act_likelihood = 1
        part_posterior.append(crp*act_likelihood)

        # Normalize the posterior
        part_posterior = np.array(part_posterior) / float(sum(part_posterior))

        if debug:
            pdb.set_trace()

        # Assign partition
        partitions[i] = int(np.random.choice(list(set(partitions)) + [max(set(partitions)) + 1],
                                             1, p=part_posterior))

        if partitions[i] not in subgoals.keys():
            subgoals[partitions[i]] = new_sg

        # Remove old subgoals
        for key in subgoals.keys():
            if key not in set(partitions):
                del subgoals[key]
        # pdb.set_trace()

    return partitions, subgoals


def bnirl_sampling_3(states, partitions, primitives, verbose=False, debug=False, eta=1):
    '''One Gibbs sampling sweep,
    run in for loop until it converges

    Args:
        states [np.array]: m x n where n is actuators and m is samples
        partitions [list]: m x 1 partition label for each sample, where label is index of state with that subgoal location
        primitives [np.array]: m x n array describing motion at sample
            e.g., primitives[0] = [1, 0, -1, 1]
                1: positive velocity or input
                0: no velocity,
                -1: negative velocity
        verbose [bool]: gives more info, for debugging
        eta [int]: hyperparameter for CRP

    Returns:
        partitions [list]: updated partitions after Gibbs sweep
        '''
    dim = len(states)

    if debug:
        pdb.set_trace()

    # For each observation
    for i, state in enumerate(states):
        if verbose:
            print('After observation %i, there are %i partitions') % (i, len(set(partitions)))

        part_posterior = []

        # For each existing partition
        for j in set(partitions):  # existing partitions
            crp = partitions.count(j) / float(dim - 1 + eta)
            act_likelihood = np.e**(np.linalg.norm(primitives[i] - rational_actor_new(states[j], state)))

            # Compute partition assignment posterior
            part_posterior.append(crp*act_likelihood)

        # Draw new subgoal uniformly from demo states
        new_sg_label = np.random.randint(0, dim)
        crp = eta / float(dim - 1 + eta)
        act_likelihood = np.e**(np.linalg.norm(primitives - rational_actor_new(states[new_sg_label], state)))
        part_posterior.append(crp*act_likelihood)

        # Normalize the posterior
        part_posterior = np.array(part_posterior) / float(sum(part_posterior))

        # Assign partition
        partitions[i] = int(np.random.choice(list(set(partitions)) + [new_sg_label],
                                             1, p=part_posterior))

    return partitions


def dp_kmeans(states, partitions, verbose=False, debug=False, lamb=1):
    '''run in for loop until it converges

    Args:
        states [np.array]: m x n where n is actuators and m is samples
        partitions [list]: m x 1 partition label for each sample
        primitives [np.array]: m x n array describing motion at sample
            e.g., primitives[0] = [1, 0, -1, 1]
                1: positive velocity or input
                0: no velocity,
                -1: negative velocity
        verbose [bool]: gives more info, for debugging
        lamb [float]: cluster distance penalty, (lambda conlicts with Python)

    Returns:
        partitions [list]: updated partitions after Gibbs sweep
        '''
    dim = len(states)

    if debug:
        pdb.set_trace()

    # For each observation
    for i, state in enumerate(states):
        # Start by recomputing subgoal locations from partition means
        sg_means = dict()
        for j in set(partitions):
            sg_mean = []
            for act in states.transpose():
                sg_mean.append(np.mean([act[k] for k in range(dim) if (partitions[k] == j)]))
            sg_means[j] = np.array(sg_mean)

        if verbose:
            print('After observation %i, there are %i partitions with means %s') % (i, len(set(partitions)), str(sg_means))

        cluster_dist = []

        # For each existing partition
        for j in set(partitions):  # existing partitions
            cluster_dist.append((j, (np.linalg.norm(sg_means[j] - state))))

        min_cluster = min(cluster_dist, key=lambda t: t[1])

        # Pick closest cluster or form new cluster
        if min_cluster[1] < lamb:
            partitions[i] = min_cluster[0]
        else:
            partitions[i] = max(set(partitions)) + 1

        if debug:
            pdb.set_trace()

    return partitions


def test_data(samples=100):
    action_set = [-1, 0, 1]
    primitives = []
    states = []
    dimm = 100

    for i in range(dimm):
        primitives.append(np.random.choice(action_set, 2))

    for i in range(dimm / 2):
        states.append(np.random.normal([2, 2], [1, 3]))

    for i in range(len(states), dimm):
        states.append(np.random.normal([-3, -4], [2, 1]))




def smooth(x, window_len=11, window='hanning'):
    """smooth the data using a window with requested size.

    This method is based on the convolution of a scaled window with the signal.
    The signal is prepared by introducing reflected copies of the signal
    (with the window size) in both ends so that transient parts are minimized
    in the begining and end part of the output signal.

    input:
        x: the input signal
        window_len: the dimension of the smoothing window; should be an odd integer
        window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
            flat window will produce a moving average smoothing.

    output:
        the smoothed signal

    example:

    t=linspace(-2,2,0.1)
    x=sin(t)+randn(len(t))*0.1
    y=smooth(x)

    see also:

    numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
    scipy.signal.lfilter

    TODO: the window parameter could be the window itself if an array instead of a string
    NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
    """

    if x.ndim != 1:
        raise ValueError, "smooth only accepts 1 dimension arrays."

    if x.size < window_len:
        raise ValueError, "Input vector needs to be bigger than window size."


    if window_len<3:
        return x


    if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
        raise ValueError, "Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'"


    s=np.r_[x[window_len-1:0:-1],x,x[-1:-window_len:-1]]
    #print(len(s))
    if window == 'flat': #moving average
        w=np.ones(window_len,'d')
    else:
        w=eval('np.'+window+'(window_len)')

    y=np.convolve(w/w.sum(),s,mode='valid')
    return y


def compute_means(partitions, states):
    '''  Computes means and mean square error of all state clusters

    Args:
        partitions
        states

    Returns:
        means (np.ndarray): k x m array of subgoal means
        mse (float): total mean squared error of clusters
    '''
    sg_set = set(partitions)
    sg_means = np.zeros((len(sg_set), states.shape[1]))
    # pred = np.zeros((states.shape))
    squared_error = []

    for i, sg in enumerate(sg_set):
        states_sg = np.array([states[j] for j in range(len(states)) if partitions[j] == sg])
        sg_means[i] = np.mean(states_sg, axis=0)
        squared_error += (((states_sg - sg_means[i])**2).tolist())

    # pdb.set_trace()
    mean_squared_error = np.mean(squared_error)

    return mean_squared_error, sg_means
