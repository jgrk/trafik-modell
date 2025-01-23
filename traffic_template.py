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
        self.laneCount = [0 for _ in range(numLanes)]
        self.v0 = v0

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
                lane_idx = rng.randint(0, numLanes)

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
            self.laneCount[lane_idx] += 1

    def getPositions(self):
        positions = {i: [] for i in range(self.numCars)}
        for lane_idx in range(self.numLanes):
            car: NewCar = self.last_cars[lane_idx]
            if car is None:
                continue
            num_cars_in_lane = self.laneCount[lane_idx]
            for _ in range(num_cars_in_lane):
                positions[car.c] += (car.x, car.lane)
                car = car.next
        return positions

    def laneSwitchTrue(self, car: NewCar, lane_num: int):
        if self.numLanes == 1:
            return False

        if lane_num < 0 or lane_num > self.numLanes - 1:
            return False

        # todo: add logic for case when switching lanes in behind the last car for that specific lane

        side_car: NewCar = self.last_cars[lane_num]
        if side_car is None:
            return True

        if side_car.next == side_car and (
                side_car.x - car.x > car.v or car.x - side_car.x > car.v
        ):
            return True

        if side_car.x > car.x:
            return False if side_car.x - car.x <= car.v else True

        while (
                side_car.next.x < car.x and side_car != self.first_cars[lane_num]
        ):  # traverse to car
            side_car = side_car.next

        if car.x - side_car.x <= side_car.v or side_car.next.x - car.x <= car.v:
            return False
        return True

    def switchLane(self, car: NewCar, lane_num: int):

        side_car = self.last_cars[lane_num]
        if car == self.last_cars[car.lane]:
            self.last_cars[car.lane] = car.next
        if car == self.first_cars[car.lane]:
            self.first_cars[car.lane] = car.prev

        if side_car is None:
            self.last_cars[lane_num] = self.first_cars[lane_num] = car

            car.next = car
            car.prev = car

            old_car_next = car.next
            old_car_prev = car.prev

            old_car_next.prev = old_car_prev  # fill in the gap in the previous lane
            old_car_next.next = old_car_next

        if side_car.x < car.x:
            while (
                    side_car.next.x < car.x and side_car != self.first_cars[lane_num]
            ):  # traverse to car
                side_car = side_car.next
            next_car = side_car.next
            prev_car = side_car
        else:
            next_car = side_car
            prev_car = side_car.prev

        old_car_next = car.next
        old_car_prev = car.prev

        car.next = next_car  # assign the pointers to the new cars in the new line
        car.prev = prev_car

        old_car_next.prev = old_car_prev  # fill in the gap in the previous lane
        old_car_next.next = old_car_next

        if car.next == self.last_cars[lane_num]:
            self.first_cars[lane_num] = car
        if car.prev == self.first_cars[lane_num]:
            self.last_cars[lane_num] = car

        self.laneCount[car.lane] -= 1
        self.laneCount[lane_num] += 1
        car.lane = lane_num  # assign new lane number for car


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
        obs.positions.append(cars.getPositions())

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
        vSum = 0
        numLanes = cars.numLanes
        # todo : Iterate using lane count instead of traversing through nodes
        for lane_idx in numLanes:
            car: NewCar = cars.last_cars[lane_idx]
            if car is None:
                continue
            num_cars_in_lane = cars.laneCount[lane_idx]

            for _ in range(cars.laneCount[lane_idx]):
                # apply logic for each car
                if cars.laneCount[lane_idx] > 1:
                    d = car.next.x - car.x

                    if car.v < self.vmax + car.lane < d:
                        car.speedUp()

                    if d <= car.v:  # rule: avoid collision
                        if cars.laneSwitchTrue(car, car.lane + 1):
                            cars.switchLane(car, car.lane + 1)
                        else:
                            car.avoidCollision(d)

                if car.v > 0 and rng.rand(1) < self.p: # Randomly slow down
                    car.slowDown()

                car.x = (car.x + car.v) % cars.roadLength
                vSum += car.v
                cars.t += 1

        return vSum / cars.roadLength


############################################################################################


def draw_cars(cars, cars_drawing):
    """Ritar bilarna och markerar radierna med tunna sträckade linjer."""
    theta = []
    r = []

    # Radier för körfälten
    lane_radii = {1: 0.6, 2: 0.9, 3: 1.2}

    # Rita de sträckade linjerna för varje radie
    for radius in lane_radii.values():
        cars_drawing.plot(
            np.linspace(0, 2 * math.pi, 100),  # Vinklar (0 till 2π)
            [radius] * 100,                   # Samma radie över hela cirkeln
            linestyle="--",                   # Sträckad linje
            linewidth=0.5,                    # Tunn linje
            color="gray",                     # Grå färg
        )

    # Räkna ut bilarnas positioner och radier
    for position, lane in zip(cars.x, cars.lanes):
        # Konvertera position till radianer
        theta.append(position * 2 * math.pi / cars.roadLength)
        # Tilldela radie beroende på körfält
        r.append(lane_radii[lane])

    # Rita bilarna
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
