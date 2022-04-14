'''
Casefile Parsing

Contains functions to make it easier for MOOS C++ code to
parse properties from the TransitionSystems YAML Files
'''

# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

# This material is based upon work supported by the Under Secretary of Defense for 
# Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any 
# opinions, findings, conclusions or recommendations expressed in this material 
# are those of the author(s) and do not necessarily reflect the views of the Under 
# Secretary of Defense for Research and Engineering.

# (C) 2021 Massachusetts Institute of Technology.

# Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

# The software/firmware is provided to you on an As-Is basis

# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS 
# Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. 
# Government rights in this work are defined by DFARS 252.227-7013 or DFARS 
# 252.227-7014 as detailed above. Use of this work other than as specifically 
# authorized by the U.S. Government may violate any copyrights that exist in this 
# work.

# SPDX-License-Identifier: BSD-3-Clause

import yaml
from math import sin, cos, radians
import numpy as np

def get_state_information(file_path, rotation_angle=None, rotation_point=None, rotation_angle_in_degrees=False, scale=None, offset=None, scale_before_offset=True, digits_precision=5):
    '''
    Parses YAML file describing the transition system. This function is used
    as a convenience to pass state information to C++ functions.

    TODO: Replace first part with call to get_yaml_dictionary() function
    
    Args:
        file_path (string): global path to the yaml input file.

        rotation_angle (double): Angle by which all points are rotated w.r.t. the point specified by rotation_point.
            NOTE: Rotation is performed _before_ performing the scale and offset operations.

        rotation_point (tuple / list): Specifies reference (x,y) point that other points are rotated around.
            Let R be the rotation matrix, p_old be an old point, and p_rot be the corresponding rotated point.
            The rotation is p_rot = R(p_old - rotation_point) + rotation_point.

        rotation_angle_in_degrees (bool): If true, the rotation_angle is in degrees. If false, rotation_angle is in radians.
                                Defaults to false.

        scale (double):  Number to scale the state point information by. For example,
                        setting scale=2 would turn a point [1,2] into [2,4].
                        If no scaling is desired, set to None.

        offset (tuple / list): Tuple / list of (x,y) offsets to be applied to all state points.
                        For example, setting offset=[1,-2] would turn a point
                        [0,0] into [1,-2].

        scale_before_offset (bool): If true, the scale is applied first, and then 
                            the offset. If false, the offset is applied first, and
                            then the scale.

        digits_precision (int): Number of digits to round to using np.around(). Prevents numbers with exponential notation.

    Returns:
        (tuple): A Tuple containing:

            - **output_points** (*list*): List containing 


    '''
    output_points = []

    with open(file_path, 'r') as file:
        string = ""
        for line in file.readlines():
            if not line.startswith('!'):
                string += line

        data = yaml.safe_load(string)

        if 'nogo_states' in data["graph"].keys():
            # The ts file has the property "nogo_states"; create the list.
            nogo_states = ', '.join(data["graph"]["nogo_states"])
        else:
            nogo_states = None


        for node in data["graph"]["nodes"].keys():
            shape = data["graph"]["nodes"][node]["shape"]
            node_string = "pts={"
            for pair in shape:
                if rotation_angle is not None:
                    pair[0], pair[1] = rotate_point(pair, rotation_angle, rotation_point, rotation_angle_in_degrees=rotation_angle_in_degrees)

                if (scale is not None) or (offset is not None):
                    if scale_before_offset:
                        if scale is not None:
                            pair[0] *= scale
                            pair[1] *= scale

                        if offset is not None:
                            pair[0] += offset[0]
                            pair[1] += offset[1]
                    else:
                        if offset is not None:
                            pair[0] += offset[0]
                            pair[1] += offset[1]

                        if scale is not None:
                            pair[0] *= scale
                            pair[1] *= scale

                node_string += f"{np.around(pair[0], digits_precision)},{np.around(pair[1], digits_precision)}: "

            # Strip off the last ": "
            node_string = node_string[:-2]
            node_string += "}, label=" + node.upper() # Upper case for MOOS
            
            output_points += [node_string]

    return output_points, nogo_states


def get_yaml_path(casefile_module, iitchs_path):
    """
    Convenience function for getting the full path to a yaml file.

    Inputs:

        casefile_module:    The casefile module. Needs to have the variable
                            `ts_filename`. Assumes the first two characters of
                            `ts_filename` are "./".

        iitchs_path:        The full path to the iitchs folder (iitchs_MOOS).
    """

    if hasattr(casefile_module, 'ts_filename'):
        yaml_path = casefile_module.ts_filename
        if yaml_path[0:1] == '.':
            # Strip off leading period
            yaml_path = yaml_path[1:]

        return (iitchs_path + "/src/" + yaml_path)
    else:
        print("\n\nError (in get_yaml_path): casefile_module does not have the variable ts_filename.")
        return None


def get_state_dictionary(file_path):
    '''
    Returns a dictionary mapping state names from the yaml casefile to numbers.

    For now, q1 -> 1, q2 -> 2, etc.
    '''
    data = get_yaml_dictionary(file_path)

    keys = data["graph"]["nodes"].keys()
    values = [ii for ii in range(1,len(keys)+1)]

    return dict(zip(keys,values))



def get_yaml_dictionary(file_path):
    '''
    Input: Absolute file path to yaml file. Use get_yaml_path to assist with this.

    Output: Dictionary object containing YAML elements.

    NOTE: This function first strips out all lines starting with an exclamation point `!`
    '''
    with open(file_path, 'r') as file:
        string = ""
        for line in file.readlines():
            if not line.startswith('!'):
                string += line

        data = yaml.safe_load(string)

    return data

def rotate_point(point, theta, rotation_center, rotation_angle_in_degrees=False):
    '''
    Rotates a point by theta w.r.t. rotation_center.

    TODO: Make this more efficient by putting in all points as batch
    '''
    if rotation_angle_in_degrees:
        theta = radians(theta)

    R =  np.array([[cos(theta), -sin(theta)],[sin(theta), cos(theta)]])
    np_point = np.array(point)
    np_center = np.array(rotation_center)

    new_point = R.dot(np_point - np_center) + np_center
    return new_point[0], new_point[1]
