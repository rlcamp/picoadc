#!/usr/bin/env python3
# before running this, do "pip3 install matplotlib" on your mac (without sudo)
# then run it with: ./serial_waterfall.py /dev/tty.usbmodem[press tab to complete]

import sys
import struct
from collections import namedtuple
import threading
import queue
import math

import fcntl
import tty
import termios
import os

import numpy as np

import matplotlib
matplotlib.rcParams['toolbar'] = 'None'

# if text gets piled on top of other text, try messing with this logic. the same settings do
# not seem to give satisfactory results on all combinations of OS and screen dpi. if someone
# knows what to do here that does the right thing unconditionally lmk
# if matplotlib.get_backend() != 'MacOSX': matplotlib.rcParams['figure.dpi'] = 300

import matplotlib.pyplot as plt

to_rgba_func = matplotlib.cm.ScalarMappable(cmap=matplotlib.colormaps['turbo']).to_rgba

def open_tty_as_stdin(path, speed=None):
    # open tty with O_NONBLOCK flag so that it doesn't hang forever at this line
    fd_tty = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    # immediately remove O_NONBLOCK because we only needed it so we could return from open()
    fcntl.fcntl(fd_tty, fcntl.F_SETFL, fcntl.fcntl(fd_tty, fcntl.F_GETFL) & ~os.O_NONBLOCK)

    tty.setraw(fd_tty)

    c_iflag, c_oflag, c_cflag, c_lflag, ispeed, ospeed, c_cc = termios.tcgetattr(fd_tty)

    if speed is not None:
        ispeed = speed
        ospeed = speed

    # ignore modem control lines and enable hupcl
    c_cflag |= termios.HUPCL | termios.CLOCAL

    termios.tcsetattr(fd_tty, termios.TCSANOW, [c_iflag, c_oflag, c_cflag, c_lflag, ispeed, ospeed, c_cc])

    os.dup2(fd_tty, 0)
    os.close(fd_tty)

def child_thread(main_thread_work):
    scale = 0.75
    offset = -256 * scale

    for line in sys.stdin:
        df_text, dt_text, levels_text = line.split(',')
        levels = np.array(bytearray.fromhex(levels_text)) * scale + offset

        packet_tuple = namedtuple('packet', [ 'levels', 'f0', 'df', 'dt'])
        main_thread_work.put(packet_tuple(levels = levels,
                                          f0 = 0,
                                          df = float(df_text),
                                          dt = float(dt_text)))

    # inform main thread that child generator has reached eof and no more input is coming
    main_thread_work.put(None)

def main():
    if len(sys.argv) > 1:
        open_tty_as_stdin(sys.argv[1], speed=(int(sys.argv[2]) if len(sys.argv) > 2 else None))

    plotdata = None
    X = 0
    Y = 0
    ax = None
    im = None
    iy = 0

    # constants you might want to fiddle with. TODO: allow main() to modify these
    clim=(-130, 0)

    # create an empty figure but don't show it yet
    fig = plt.figure()

    # thread-safe fifo between rx thread and main thread
    main_thread_work = queue.Queue()

    pth = threading.Thread(target=child_thread, args=(main_thread_work,))
    pth.start()

    # event loop which dequeues work from other threads that must be done on main thread
    while True:
        try:
            packet = main_thread_work.get(timeout=0.016)
        except queue.Empty:
            fig.canvas.flush_events()
            continue
        if packet is None: break

        # do this setup stuff on the first input
        if not X:
            X = packet.levels.shape[0]
            Y = (3 * X) // 4 # plot will have an aspect ratio of 3/4
            plotdata = np.zeros((2 * Y, X, 4), dtype=np.uint8)

            ax = fig.add_subplot(1, 1, 1)

            f0 = packet.f0

            df = packet.df
            ff = f0 + X * df
            xextent = (f0 / 1e3, ff / 1e3)

            yextent = (0, packet.dt * Y)

            im = ax.imshow(plotdata[0:Y, :, :],
                origin='lower',
                extent=[xextent[0], xextent[1], yextent[0], yextent[1]],
                interpolation='none',
                aspect=(((xextent[1] - xextent[0]) * Y) / ((yextent[1] - yextent[0]) * X)), animated=True)

            # label the y axis for the subplots on the left side
            ax.set(ylabel='Time (s) in past')

            # label the x axis for the subplots on the bottom
            ax.set(xlabel='Frequency (kHz)')

            fig.tight_layout(pad=1.5)
            fig.show()

            fig.canvas.blit(fig.bbox)
            fig.canvas.draw()

        # if not the first call, sanity check that number of bins has not changed
        elif packet.levels.shape[0] != X:
            raise RuntimeError('consecutive packets have different numbers of bins (%u != %u)' % levels.shape[0], X)

        # convert the values in dB for the new row of pixels to rgba values
        bins_rgba = to_rgba_func(np.clip((packet.levels - clim[0]) / (clim[1] - clim[0]), 0, 1), bytes=True, norm=False)

        # insert the new row of pixels into two places within the doubled ring buffer, so that a
        # contiguous slice of it can always be plotted, ending at the most recent row
        plotdata[iy + 0, :, :] = bins_rgba
        plotdata[iy + Y, :, :] = bins_rgba

        # only do the expensive redraw when we are caught up
        if main_thread_work.empty():
            # update which subset of the doubled ring buffer will be shown
            im.set_data(plotdata[iy:(iy + Y), :])

            ax.draw_artist(im)
            fig.canvas.blit(ax.bbox)
            fig.canvas.flush_events()

        # advance the ring buffer cursor (decrements w/ wraparound, as newest time is at bottom)
        iy = (iy + Y - 1) % Y

    # if we get here, we got to eof on stdin
    pth.join()

main()
