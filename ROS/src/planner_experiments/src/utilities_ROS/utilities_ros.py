import numpy as np
from std_msgs.msg import Float32MultiArray,MultiArrayDimension
from planner_experiments.msg import agent_info
import warnings
from plan_lib.utilities import load_var

def serialise_np(data, header = None):
    # Code the msg into Float32MultiArray
    msg = agent_info()

    # For each variable we add a filed to msg
    for i,field in enumerate(data):

        proxy = Float32MultiArray()
        proxy.data = field

        # This is almost always zero there is no empty padding at the start of your data
        proxy.layout.data_offset = 0
        proxy.data = field.flatten()

        # create two dimensions in the dim array
        proxy.layout.dim = [MultiArrayDimension() for _ in range(0,field.ndim)]
        dims = field.shape

        # for each dimension we add a layout so we now how to unpack it
        for k in range(0,len(proxy.layout.dim)):
            # dim[0] is the vertical dimension of your matrix
            proxy.layout.dim[k].label = "dim " + str(k)
            proxy.layout.dim[k].size = dims[k]
            proxy.layout.dim[k].stride = np.prod(dims[k::])

        msg.data.append(proxy)

    return msg

def deserialise_np(msg):
    # To unpack the msg we resize the raw data with the dimensions in the layouts
    data = ['']*len(msg.data)
    for i,field in enumerate(msg.data):
        size = []
        for d in field.layout.dim:
            size.append(d.size)
        data[i] = np.asarray(field.data).reshape(tuple(size))

    return data

def get_lambdasROS(settings, id):
    n_agents = settings["n_agents"]
    N = settings["N"]
    path = settings["lb_path"] + "pck/" + str(id) + "/" + "ini_lambdas.pkl"
    lambdas = np.zeros((n_agents, N))

    try:
        lambdas[:, :] = load_var(path)
    except Exception as e:
        print(e)
        msg = "unable to load lambdas, defaulting to 0s"
        warnings.warn((msg))

    return lambdas