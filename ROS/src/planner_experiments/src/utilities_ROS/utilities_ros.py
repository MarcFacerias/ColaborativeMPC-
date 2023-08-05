import numpy as np
from std_msgs.msg import Float32MultiArray
from planner_experiments.msg import agent_info


def serialise_np(data, header = None):

    msg = agent_info()

    for i,field in enumerate(data):

        msg.data[i].data = field

        # This is almost always zero there is no empty padding at the start of your data
        msg.data[i].layout.data_offset = 0
        msg.data[i].data = field.flatten() # TODO: check this
        # create two dimensions in the dim array
        msg.data[i].layout.dim = [Float32MultiArray() for _ in range(0,field.ndim)]
        dims = field.shape()

        for k in range(0,msg.data[i].layout.dim):
            # dim[0] is the vertical dimension of your matrix
            msg.data[i].layout.dim[k].label = "dim" + str()
            msg.data[i].layout.dim[k].size = dims[k]
            msg.data[i].layout.dim[k].stride = np.sum(dims[k::])

    return msg

def deserialise_msg(msg):

    data = ['']*len(msg)

    for i,field in enumerate(msg.data):

        data[i] = field

    return data
