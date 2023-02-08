import matplotlib
from matplotlib import colors
from matplotlib.backends import backend_agg

%pylab inline

def sense(x):
    return x
    
def simulate(Δt, x, u):
    x += Δt * u
    return x
    
def control(t, y, x):
    k = 1
    ux = k*(2-x[0])
    uy = k*(10-x[1])
    return array([ux, uy])

tf = 10.
Δt = 0.01    # Time step
time = linspace(0.,tf, int(tf / Δt) + 1)  # Time interval


# Initial conditions
x = array([4., 5.])
x_log = [copy(x)]

for t in time:
    y = sense(x)
    u = control(t, y, x)    
    x = simulate(Δt, x, u)
    x_log.append(copy(x))
    
x_log = array(x_log)
time = numpy.append(time, tf)

grid()
plot(time, x_log[:,0])
plot(time, x_log[:,1])
