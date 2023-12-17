import numpy as np


def run_mlp(obs, weights, activation_function="tanh"):
    ''' 
    Run multilayer perceptron using numpy
    
    Parameters
    ----------
    obs : ndarray
        float32 array with neural network inputs
    weights : list
        list of MLP layers of type (bias, kernel)
    activation_function : str
        activation function for hidden layers
        set to "none" to disable
    '''

    obs = obs.astype(np.float32, copy=False)
    out = obs

    for w in weights[:-1]: # for each hidden layer
        out = np.matmul(w[1],out) + w[0]
        if activation_function == "tanh":
            np.tanh(out, out=out) 
        elif activation_function != "none":
            raise NotImplementedError
    return np.matmul(weights[-1][1],out) + weights[-1][0] # final layer