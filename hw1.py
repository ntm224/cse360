import matplotlib
from matplotlib import colors
from matplotlib.backends import backend_agg

%pylab inline

def sense(x):
    return x
    
def simulate(Δt, x, u):
    x += Δt * u
    return x
    
def control(t, y):
    ### WRITE YOUR CONTROL POLICY HERE:
    ux = -4*sin(t)
    uy = 2*cos(t)
    return array([ux, uy])
    
tf = 10.
Δt = 0.01    # Time step
time = linspace(0.,tf, int(tf / Δt) + 1)  # Time interval


# Initial conditions
x = array([7., 2.])
x_log = [copy(x)]

for t in time:
    y = sense(x)
    u = control(t, y)    
    x = simulate(Δt, x, u)
    x_log.append(copy(x))
    
x_log = array(x_log)

grid()
plot(x_log[:,0], x_log[:,1])
