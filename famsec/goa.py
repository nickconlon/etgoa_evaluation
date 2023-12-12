"""Tools for competency computation."""

import numpy as np


def general_oa(data, outcome_partitions, z_star):
    """Calculate generalized Outcome Assessment for a competency bound on an
    ordinal set of data.

    Args:
        data (numpy array): Singular outcomes of trajectories, organized s.t. larger
            values are better outcomes.
        outcome_partitions (numpy array): Lower limits on equivalence classes of outcomes.
        z_star (int): Threshold for which we desire outcome bins above (inclusive).

    Returns:
        float: GOA for user specified condition.
    """

    # get # of bins
    num_bins = len(outcome_partitions)

    if z_star < 1 or z_star > num_bins:
        raise Exception("z* must be an integer between 1 and number of bins")

    # ranked z-domain
    z_domain = []
    for i in range(1, num_bins):
        z_i = []
        z_i.append(
            [
                val
                for val in data
                if outcome_partitions[i - 1] < val <= outcome_partitions[i]
            ]
        )
        z_domain.append(z_i)

    # estimate the probabilities in each bin
    p_z = []
    for current_bin in z_domain:
        p_z.append(len(current_bin[0]) / len(data))

    # compute UPM/LPM
    d_lpm = 0
    d_upm = 0

    for i in range(1, len(z_domain) + 1):
        if i < z_star:
            d_lpm += (z_star - i) * p_z[i - 1]
        elif i >= z_star:
            d_upm += (i - z_star + 1) * p_z[i - 1]

    # compute GOA
    if d_lpm == 0:
        outcome_assessment = 1
    elif d_upm == 0:
        outcome_assessment = -1
    else:
        outcome_assessment = 2 / (1 + np.exp(-np.log(d_upm / d_lpm))) - 1

    return outcome_assessment


def linear_oa(data, outcome_partitions, z_star):
    """Calculate generalized Outcome Assessment for a competency bound on an
    ordinal set of data.

    Args:
        data (numpy array): Singular outcomes of trajectories, organized s.t. larger
            values are better outcomes.
        outcome_partitions (numpy array): Lower limits on equivalence classes of outcomes.
        z_star (int): Threshold for which we desire outcome bins above (inclusive).

    Returns:
        float: GOA for user specified condition.
    """

    # get # of bins
    num_bins = len(outcome_partitions)

    if z_star < 1 or z_star > num_bins:
        raise Exception("z* must be an integer between 1 and number of bins")

    # ranked z-domain
    z_domain = []
    for i in range(1, num_bins):
        z_i = []
        z_i.append(
            [
                val
                for val in data
                if outcome_partitions[i - 1] < val <= outcome_partitions[i]
            ]
        )
        z_domain.append(z_i)

    # estimate the probabilities in each bin
    p_z = []
    for current_bin in z_domain:
        p_z.append(len(current_bin[0]) / len(data))

    # compute UPM/LPM
    d_lpm = 0
    d_upm = 0

    for i in range(1, len(z_domain) + 1):
        if i < z_star:
            d_lpm += (z_star - i) * p_z[i - 1]
        elif i >= z_star:
            d_upm += (i - z_star + 1) * p_z[i - 1]
    x = 0.5
    outcome_assessment = 2 / (1 + np.exp(-x * (d_upm - d_lpm))) - 1

    return outcome_assessment


def semantic_label_color(outcome_assessment):
    """Apply semantic labeling color scheme to an outcome assessment.

    Args:
        x0 (float): Computed utcome assessment value

    Returns:
        String: Color for outcome assessment.
    """

    #labels = {-1: "red", -0.5: "salmon", -0.1: "yellow", 0.1: "yellowgreen", 0.5: "green"}
    labels = {0: "red", 0.1: "salmon", 0.3: "yellow", 0.7: "yellowgreen", 0.9: "green"}
    value = labels[max([val for val in labels if val <= outcome_assessment])]

    return value


def semantic_label_text(outcome_assessment):
    """Apply semantic labeling text scheme to an outcome assessment.

    Args:
        x0 (float): Computed utcome assessment value

    Returns:
        String: Semantic label for outcome assessment.
    """

    labels = {0: "Highly Unlikely", 0.1: "Unlikely", 0.3: "Even Chance", 0.7: "Likely", 0.9: "Highly Likely"}
    value = labels[max([val for val in labels if val <= outcome_assessment])]

    return value
