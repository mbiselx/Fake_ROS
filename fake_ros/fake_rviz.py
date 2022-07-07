import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import fake_ros.fake_rospy as rospy


class Viz():
    """
    this is obviously not rviz, just a live plotter for data
    """
    def __init__(self, xmin=-1, xmax=4, ymin=-3, ymax=3):
        self.fig, self.ax = plt.subplots(1, 1)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(xmin, xmax)
        self.ax.set_ylim(ymin, ymax)
        self.ax.axis('off')
        self.fig.tight_layout()

        # set interactive plotting and stuff
        plt.ion()
        plt.show()
        plt.pause(.001)

        #
        self.fig.canvas.mpl_connect('close_event', self.onclose)
        self.is_running = True

        # cache the background
        self.background = self.fig.canvas.copy_from_bbox(self.ax.bbox)


        # add artists :
        self.artists = list()

    def add_artist(self, artist):
        self.artists.append(artist)

    def update(self):
        if self.is_running :
            # restore background
            self.fig.canvas.restore_region(self.background)

            # redraw just the artists
            for artist in self.artists :
                self.ax.draw_artist(artist)

            # fill in the axes rectangle
            self.fig.canvas.blit(self.ax.bbox)

            # draw and pause to update plot
            plt.draw()
            plt.pause(0.001)

        else :
            raise rospy.ROSInterruptException

    def onclose(self, event):
        self.is_running = False

class VizPointCloud():
    """scatter-plots a numpy array of points of shape (:,2) """

    def __init__(self, visualizer=None):

        if visualizer :
            self.visualizer = visualizer
        else :
            self.visualizer = Viz()


        # create the artist
        self._artist = self.visualizer.ax.plot([], [], 'o')[0]
        self.visualizer.add_artist(self._artist)


    def draw(self, points):
        # set new data
        if points.shape[0] > 0 and points.shape[1] == 2 :
            self._artist.set_data(points[:,0], points[:,1])


    def update(self):
        self.visualizer.update()

class VizVector3():
    """ plots an arrow patch centered at 0/0 """

    def __init__(self, visualizer=None):

        if visualizer :
            self.visualizer = visualizer
        else :
            self.visualizer = Viz()


        # create the artist
        self._artist = self.visualizer.ax.arrow(0, 0, 0, 0)
        self.visualizer.add_artist(self._artist)




    def draw(self, x=0, y=0, z=0):
        self._artist.remove()
        new_artist = self.visualizer.ax.arrow(0, 0, x, y)

        for i, artist in enumerate(self.visualizer.artists) :
            if artist is self._artist :
                self.visualizer.artists[i] = new_artist
        self._artist = new_artist



    def update(self):
        self.visualizer.update()
