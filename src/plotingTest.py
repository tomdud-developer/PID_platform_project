import tkinter
from functools import partial
from multiprocessing import Process, Queue
from turtle import delay
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure

import numpy as np
import time

def ploting(queue):
    root = tkinter.Tk()
    root.wm_title("Embedding in Tk")

    fig = Figure(figsize=(5, 4), dpi=100)
    t = np.arange(0, 3, .01)
    ax = fig.add_subplot(111)
    ax.plot(t, 2 * np.sin(2 * np.pi * t))
    canvas = FigureCanvasTkAgg(fig, master=root)  # A tk.DrawingArea.
    canvas.draw()
    canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)

    toolbar = NavigationToolbar2Tk(canvas, root)
    toolbar.update()
    canvas.get_tk_widget().pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=1)


    def on_key_press(event):
        print("you pressed {}".format(event.key))
        key_press_handler(event, canvas, toolbar)


    canvas.mpl_connect("key_press_event", on_key_press)


    def _quit():
        root.quit()     # stops mainloop
        root.destroy()  # this is necessary on Windows to prevent
                        # Fatal Python Error: PyEval_RestoreThread: NULL tstate


    button = tkinter.Button(master=root, text="Quit", command=partial(writing, queue, ax))
    button.pack(side=tkinter.BOTTOM)

    tkinter.mainloop()

def writing(queue, ax):
    ax.clear()
    t = np.arange(0, 3, .01)
    ax.plot(t, 2 * np.cos(2 * np.pi * t))

    """
    while True:
        if not queue.empty():
            print(queue.get())
    """

if __name__ == '__main__':
    queue = Queue()
    plot_process = Process(target = ploting, args = (queue, ))
    plot_process.start()
    i = 0
    while True:
        queue.put(i)
        i+=1
        time.sleep(0.5)

    plot_process.join()
