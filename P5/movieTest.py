import matplotlib.pyplot as plt
import numpy as np
from moviepy.video.io.bindings import mplfig_to_npimage
import moviepy.editor as mpy

# DRAW A FIGURE WITH MATPLOTLIB

x = np.array([1, 2, 3, 4, 5, 6]) /10
y = np.array([1, 2, 3, 4, 5, 6]) /10

x2 = np.array([1, 2, 3, 4, 5, 6]) /10
y2 = np.array([6, 5, 4, 3, 2, 1]) /10

duration = x.shape[0]/10

fig_mpl, ax = plt.subplots(1,figsize=(5,5), facecolor='white')

#xx = np.linspace(-2,2,200) # the x vector
#zz = lambda d: np.sinc(xx**2)+np.sin(xx+d) # the (changing) z vector

ax.set_title("Lille Plutt on tour")

ax.set_ylim(0, 7)
ax.set_xlim(0, 10)

#line, = ax.plot(xx, zz(0), lw=3)
point1, = ax.plot(x, y, "o", c="r")
point2, = ax.plot(x2, y2, "o", c="g")

# ANIMATE WITH MOVIEPY (UPDATE THE CURVE FOR EACH t). MAKE A GIF.

def make_frame_mpl(t):
    #line.set_ydata( zz(2*np.pi*t/duration))  # <= Update the curve
    print("t: ",t)
    b = int(t*10)
    point1.set_ydata( y[b])  # <= Update the curve
    point1.set_xdata( x[b])  # <= Update the curve
    point2.set_ydata( y2[b])
    point2.set_xdata( x2[b])
    return mplfig_to_npimage(fig_mpl) # RGB image of the figure

animation =mpy.VideoClip(make_frame_mpl, duration=duration)
animation.write_gif("sinc_mpl.gif", fps=10)