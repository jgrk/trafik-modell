#!/bin/python3

# Template for traffic simulation
# BH, MP 2021-11-15, latest version 2024-11-08.

"""
    This template is used as backbone for the traffic simulations.
    Its structure resembles the one of the pendulum project, that is you have:
    (a) a class containing the state of the system and its parameters
    (b) a class storing the observables that you want then to plot
    (c) a class that propagates the state in time (which in this case is discrete), and
    (d) a class that encapsulates the aforementioned ones and performs the actual simulation
    You are asked to implement the propagation rule(s) corresponding to the traffic model(s) of the project.
"""

from scipy.optimize import curve_fit
import math
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy.random as rng
import numpy as np
from scipy.stats import sem

import matplotlib


class NewCar:
    def __init__(self, x, v, c, lane, next=None, prev=None):
        self.next = next
        self.prev = prev
        self.x = x
        self.v = v
        self.c = c
        self.lane = lane

    def change_lane(self, next, prev, overTaking: bool = True):
        """
        Change lane of the car
        :param overTaking: Increases lane number if true, else decreases lane number
        :return:
        """
        if overTaking:
            self.lane += 1
        else:
            self.lane -= 1

        self.next = next
        self.prev = prev

    def speedUp(self):
        """
        Increase velocity by one unit
        :return:
        """
        self.v += 1

    def slowDown(self):
        """
        Decrease velocity by one unit
        :return:
        """
        self.v -= 1

    def avoidCollision(self, d):
        """
        Collision avoidance
        :param d: Distance to car ahead
        :return: None
        """
        self.v = d - 1


class Cars:
    """Class for the state of a number of cars"""

    def __init__(self, numCars=5, roadLength=20, v0=1, numLanes=1):
        self.numCars = numCars
        self.roadLength = roadLength
        self.t = 0
        self.cars = []
        self.numLanes = numLanes
        self.last_cars = [None for _ in range(numLanes)]
        self.first_cars = [None for _ in range(numLanes)]

        for i in range(0, numCars):
            # Set the initial position for each car such that cars are evenly spaced.
            x = i * (roadLength // numCars)
            v = v0
            c = i
            if numLanes == 1:
                lane_idx = 0
            elif numLanes == 2:
                randint = rng.randint(1, 100)
                if randint < 10:
                    lane_idx = 1
                else:
                    lane_idx = 0
            else:
                lane_idx = rng.randint(0, numLanes - 1)

            new_car = NewCar(x=x, v=v, c=c, lane=lane_idx)

            if self.last_cars[lane_idx] is None:
                self.last_cars[lane_idx] = self.first_cars[lane_idx] = new_car
                new_car.next = new_car
                new_car.prev = new_car
            else:
                # make old first car point to the new first car
                # new first car points to the last car
                last_car = self.last_cars[lane_idx]
                first_car = self.first_cars[lane_idx]

                first_car.next = new_car
                new_car.prev = first_car

                last_car.prev = new_car
                new_car.next = last_car
                self.first_cars[lane_idx] = new_car



class Observables:
    """Class for storing observables"""

    def __init__(self):
        self.time = []  # list to store time
        self.flowrate = []  # list to store the flow rate
        self.positions = []  # list to store positions of all cars at each time step
        self.lanes = []

class BasePropagator:

    def __init__(self):
        return

    def propagate(self, cars, obs):
        """Perform a single integration step"""

        fr = self.timestep(cars, obs)

        # Append observables to their lists
        obs.time.append(cars.t)
        obs.flowrate.append(fr)
        obs.positions.append(list(cars.x))
        obs.lanes.append(list(cars.lanes))

    def timestep(self, cars, obs):
        """Virtual method: implemented by the child classes"""

        pass


class ConstantPropagator(BasePropagator):
    """
    Cars do not interact: each position is just
    updated using the corresponding velocity
    """

    def timestep(self, cars, obs):
        for i in range(cars.numCars):
            cars.x[i] = (cars.x[i] + cars.v[i]) % cars.roadLength
        cars.t += 1
        return 0


class MyPropagator(BasePropagator):

    def __init__(self, vmax, p):
        BasePropagator.__init__(self)
        self.vmax = vmax
        self.p = p

    def timestep(self, cars, obs):
        for i in range(cars.numCars):
            # Determine the distance to the next car
            d = cars.distance(i)

            # Rule 2: Deceleration - Slow down to avoid collision if the cars are in the same lane
            if d <= cars.v[i] and cars.lanes[i] == cars.lanes[i + 1]:
                if cars.laneOccupied(car_idx=i, lane_idx=cars.lanes[i] + 1) or cars.lanes[i] == cars.numLanes:  # Slow down if left lane occupied or does not exist
                    cars.v[i] = d - 1
                else:  # if overtaking available increase lane number and increase speed
                    cars.lanes[i] += 1
                    cars.v[i] += 1

            if cars.lanes[i] == 1:
                # Rule 1: Acceleration - Increase speed if possible
                if cars.v[i] < self.vmax:
                    cars.v[i] += 1

                if cars.v[i] > self.vmax:
                    cars.v[i] -= 1

            # rule for middle lane
            if cars.lanes[i] > 1:
                if cars.v[i] < self.vmax + cars.lanes[i] - 1:
                    cars.v[i] += 1

                # return to inner lane if possible
                if cars.laneOccupied(car_idx=i, lane_idx=cars.lanes[i] - 1) == False:
                    cars.lanes[i] -= 1

            # Rule 3: Randomization - Randomly slow down
            if cars.v[i] > 0 and rng.rand() < self.p:
                cars.v[i] -= 1

            # Update position
            cars.x[i] = (cars.x[i] + cars.v[i]) % cars.roadLength

        # Update time
        cars.t += 1

        # Calculate flow rate (number of cars passing a point per time step)
        return sum(cars.v) / cars.roadLength


############################################################################################


def draw_cars(cars, cars_drawing):
    """Used later on to generate the animation"""
    theta = []
    r = []

    for position in cars.x:
        # Convert to radians for plotting only (do not use radians for the simulation!)
        theta.append(position * 2 * math.pi / cars.roadLength)
        r.append(1)

    return cars_drawing.scatter(theta, r, c=cars.c, cmap="hsv")


def animate(framenr, cars, obs, propagator, road_drawing, stepsperframe):
    """Animation function which integrates a few steps and return a drawing"""

    for it in range(stepsperframe):
        propagator.propagate(cars, obs)

    return (draw_cars(cars, road_drawing),)


class Simulation:

    def reset(self, cars=Cars()):
        self.cars = cars
        self.obs = Observables()

    def __init__(self, cars=Cars()):
        self.reset(cars)

    def plot_observables(self, title="simulation"):
        plt.clf()
        plt.title(title)
        plt.plot(self.obs.time, self.obs.flowrate)
        plt.xlabel("time")
        plt.ylabel("flow rate")
        plt.savefig(title + ".pdf")
        plt.show()

    def plot_density_vs_flowrate(
            self, densities, flowrates, title="fundamental_diagram"
    ):
        plt.clf()
        plt.title(title)
        plt.plot(densities, flowrates, marker="o")
        plt.xlabel("Density (cars per road length)")
        plt.ylabel("Flow rate")
        plt.savefig(title + ".pdf")
        plt.show()

    def plot_positions_vs_time(self, title="positions_vs_time"):
        plt.clf()
        plt.title(title)
        for i in range(self.cars.numCars):
            positions = [pos[i] for pos in self.obs.positions]
            plt.scatter(self.obs.time, positions, label=f"Car {i}", s=10)
        plt.xlabel("Time")
        plt.ylabel("Position on road")
        plt.legend()
        plt.savefig(title + ".pdf")
        plt.show()

    def getAverageFlowrate(self, propagator, numsteps=200):
        for it in range(numsteps):
            propagator.propagate(self.cars, self.obs)

        return sum(self.obs.flowrate) / numsteps

    # Run without displaying any animation (fast)
    def run(
            self,
            propagator,
            numsteps=200,  # final time
            title="simulation",  # Name of output file and title shown at the top
    ):

        for it in range(numsteps):
            propagator.propagate(self.cars, self.obs)

        # self.plot_observables(title)
        # self.plot_positions_vs_time(title + "_positions")

    # Run while displaying the animation of bunch of cars going in circle (slow-ish)
    def run_animate(
            self,
            propagator,
            numsteps=200,  # Final time
            stepsperframe=1,  # How many integration steps between visualising frames
            title="simulation",  # Name of output file and title shown at the top
    ):

        numframes = int(numsteps / stepsperframe)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="polar")
        ax.axis("off")
        # Call the animator, blit=False means re-draw everything
        anim = animation.FuncAnimation(
            plt.gcf(),
            animate,  # init_func=init,
            fargs=[self.cars, self.obs, propagator, ax, stepsperframe],
            frames=numframes,
            interval=50,
            blit=True,
            repeat=False,
        )
        plt.show()

        # If you experience problems visualizing the animation and/or
        # the following figures comment out the next line
        # plt.waitforbuttonpress(30)

        self.plot_observables(title)
        self.plot_positions_vs_time(title + "_positions")
