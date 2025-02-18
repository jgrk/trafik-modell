#!/bin/python3

# Template for traffic simulation
import time
from typing import Dict, Tuple

from scipy.optimize import curve_fit
import math
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy.random as rng
import numpy as np
from typing import Iterable
from time import sleep

import matplotlib

# Uncomment this if you are running with the pycharm debugger
# matplotlib.use('Agg')


class NewCar:
    def __init__(self, x, v, c, lane):
        self.next = None
        self.prev = None
        self.x = x
        self.v = v
        self.c = c
        self.lane = lane

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

    def avoidCollision(self, d: int):
        """
        Collision avoidance
        :param d: Distance to car ahead
        :type d: int:
        :return: None
        :rtype: None
        """
        self.v = d - 1


class Cars:
    """
    Groups a set of NewCar objects and sets simulation parameters

    Includes functionality for each car when considered in traffic

    Used together with Simulation class as a parameter.

    Example:
        >>> cars = Cars()
        >>> sim = Simulation(cars=cars)
    """

    def __init__(
        self,
        numCars=5,
        roadLength=20,
        v0=1,
        numLanes=1,
        laneSwitchAllowed: bool = True,
        carDensity=None,
    ):
        self.roadLength = roadLength
        if carDensity is None:
            self.numCars = numCars
        else:
            self.numCars = int(carDensity * roadLength)
        self.t = 0
        self.cars = []
        self.numLanes = numLanes
        self.last_cars: list[NewCar | None] = [None for _ in range(numLanes)]
        self.first_cars: list[NewCar | None] = [None for _ in range(numLanes)]
        self.laneCount = [0 for _ in range(self.numLanes)]
        self.v0 = v0
        self.c = [i for i in range(self.numCars)]
        self.laneSwitchAllowed = laneSwitchAllowed
        self.numLaneChanges = 0

        for i in range(self.numCars):
            lane_idx = i % self.numLanes
            self.laneCount[lane_idx] += 1

        car_id = 0
        for lane_idx in range(self.numLanes):
            # Set the initial position for each car such that cars are evenly spaced.
            carsInLane = self.laneCount[lane_idx]
            for i in range(carsInLane):
                x = i * (roadLength // carsInLane)
                v = v0
                c = car_id
                car_id += 1

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

    def getPositions(self) -> dict[int, tuple[int, int]]:
        positions = {i: (0, 0) for i in range(self.numCars)}
        for lane_idx in range(self.numLanes):
            car: NewCar = self.last_cars[lane_idx]
            if car is None:
                continue
            num_cars_in_lane = self.laneCount[lane_idx]
            for _ in range(num_cars_in_lane):
                positions[car.c] = (car.x, car.lane)
                car = car.next
        return positions

    def laneSwitchTrue(self, car: NewCar, lane_num: int) -> bool:
        """
        Check if lane switch is possible

        :param car: NewCar object
        :type car: NewCar
        :param lane_num: Target lane
        :type lane_num: int
        :return: True if lane switch is possible
        :rtype: bool
        """

        if self.laneSwitchAllowed is False:
            return False

        if self.numLanes == 1:
            return False

        if lane_num < 0 or lane_num > self.numLanes - 1:
            return False

        side_car: NewCar = self.last_cars[lane_num]
        if side_car is None:
            return True

        if side_car.next == side_car and (
            (side_car.x - car.x) % self.roadLength > car.v
            or (car.x - side_car.x) % self.roadLength > car.v
        ):
            return True

        if side_car.x > car.x:
            if (side_car.x - car.x) % self.roadLength <= car.v or (
                self.roadLength - side_car.prev.x + car.x
            ) % self.roadLength <= side_car.prev.x:
                return False
            else:
                return True

        while (
            side_car.next.x < car.x and side_car != self.first_cars[lane_num]
        ):  # traverse to car
            side_car = side_car.next

        if (car.x - side_car.x) % self.roadLength <= side_car.v or (
            side_car.next.x - car.x
        ) % self.roadLength <= car.v:
            return False
        return True

    def switchLane(self, car: NewCar, lane_num: int):
        """
        Perform lane switch on car

        :param car: NewCar object
        :type car: NewCar
        :param lane_num: Target lane
        :type lane_num: int
        :return: None
        :rtype: None
        """

        side_car: NewCar | None = self.last_cars[lane_num]
        if self.laneCount[car.lane] == 1:
            self.last_cars[car.lane] = self.first_cars[car.lane] = None
        else:
            if car == self.last_cars[car.lane]:
                self.last_cars[car.lane] = car.next
            if car == self.first_cars[car.lane]:
                self.first_cars[car.lane] = car.prev

        if side_car is None:
            # mark the car as first and last since only car in the new lane
            self.last_cars[lane_num] = self.first_cars[lane_num] = car

            # save pointers to next and prev cars in old lane
            old_car_next = car.next
            old_car_prev = car.prev

            # assign pointers to itself, since only car in the new lane
            car.next = car
            car.prev = car

            # fill gap in old lane
            if self.laneCount[car.lane] > 1:
                old_car_next.prev = old_car_prev  # fill in the gap in the previous lane
                old_car_prev.next = old_car_next

            # set new lane counts and update lane number for car
            self.laneCount[car.lane] -= 1
            self.laneCount[lane_num] += 1
            car.lane = lane_num

            return None

        if self.laneCount[lane_num] == 1:
            next_car = prev_car = side_car

            # save old pointers
            old_car_next = car.next
            old_car_prev = car.prev

            # assign pointers of the two cars to each other
            car.next = next_car
            side_car.next = car
            car.prev = prev_car
            side_car.prev = car

            # fill gap in old lane
            if self.laneCount[car.lane] > 1:
                old_car_next.prev = old_car_prev
                old_car_prev.next = old_car_next

            if car.x < side_car.x:
                self.first_cars[lane_num] = side_car
                self.last_cars[lane_num] = car
            else:
                self.first_cars[lane_num] = car
                self.last_cars[lane_num] = side_car

            # update lane count and lane number
            self.laneCount[car.lane] -= 1
            self.laneCount[lane_num] += 1
            car.lane = lane_num

            return None

        if side_car.x < car.x:
            while (
                side_car.next
                and side_car.next.x < car.x
                and side_car != self.first_cars[lane_num]
            ):  # traverse to car
                side_car = side_car.next
            next_car = side_car.next
            prev_car = side_car
            if side_car == self.first_cars[lane_num]:
                self.first_cars[lane_num] = car
        else:
            next_car = side_car
            prev_car = side_car.prev
            self.last_cars[lane_num] = car

        old_car_next = car.next
        old_car_prev = car.prev

        car.next = next_car  # assign the pointers to the new cars in the new line
        car.prev = prev_car
        prev_car.next = car
        next_car.prev = car

        if self.laneCount[car.lane] > 1:
            old_car_next.prev = old_car_prev  # fill in the gap in the previous lane
            old_car_prev.next = old_car_next

        self.laneCount[car.lane] -= 1
        self.laneCount[lane_num] += 1
        car.lane = lane_num  # assign new lane number for car

        return None

    def healthy(self):
        for lane_idx in range(self.numLanes):
            car_count = 0
            if self.last_cars[lane_idx] is None:
                assert self.laneCount[lane_idx] == car_count
                continue

            assert self.first_cars[lane_idx].next == self.last_cars[lane_idx]
            assert self.last_cars[lane_idx].prev == self.first_cars[lane_idx]
            car = self.last_cars[lane_idx]

            for _ in range(self.laneCount[lane_idx]):
                if self.laneCount[lane_idx] != 1:
                    try:
                        assert car.next != car
                    except:
                        print("failure")

                else:
                    assert car.next == car
                    assert car.prev == car

                if car != self.first_cars[car.lane] and self.laneCount[car.lane] > 1:
                    try:
                        assert car.next.x > car.x
                    except:
                        if self.laneCount[car.lane] == 2:
                            self.first_cars[car.lane] = car
                            self.last_cars[car.lane] = car.next
                        else:
                            print("failure")

                assert car.lane == lane_idx
                car_count += 1
                car = car.next

            assert car_count == self.laneCount[lane_idx]

        return True


class Observables:
    """Class for storing observables"""

    def __init__(self):
        self.time = []
        self.flowrate = []
        self.positions = []
        self.numLaneChanges = []


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
        obs.numLaneChanges.append(cars.numLaneChanges)

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
    """
    Propagator with lane switching logic

    Used together with Cars and Simulation.

    Example:
        >>> cars = Cars()
        >>> sim = Simulation(cars=cars)
        >>> prop = MyPropagator()
        >>> sim.run(propagator=prop)
    """

    def __init__(self, vmax, p):
        BasePropagator.__init__(self)
        self.vmax = vmax
        self.p = p

    def timestep(self, cars: Cars, obs: Observables):
        if cars.numCars >= cars.roadLength * cars.numLanes:
            return 0

        visited = set()
        vSum = 0
        numLanes = cars.numLanes
        cars.numLaneChanges = 0

        for lane_idx in range(numLanes):
            lane_swap = False
            car: NewCar = cars.last_cars[lane_idx]
            if car is None:
                continue
            num_cars_in_lane = cars.laneCount[lane_idx]

            # apply logic for each car
            for _ in range(num_cars_in_lane):
                # Car might already been visited if it has changed lane
                if car in visited:
                    print(f"skipping {car}, already visited")
                    car = car.next
                    continue

                visited.add(car)

                # needs to save pointer if car changes lane
                car_next = car.next
                if cars.laneCount[car.lane] > 1:
                    if car == cars.first_cars[lane_idx]:
                        d = cars.roadLength - car.x + car.next.x
                    else:
                        d = car.next.x - car.x
                    d = d % cars.roadLength
                    if d < 0:
                        print(d)
                else:
                    d = cars.roadLength

                    # speed up if possible, outer lanes has higher max speeds
                if car.v < self.vmax + car.lane and car.v < d:
                    print(f"car {car.c} speed up v = {car.v} -> {car.v + 1}")
                    car.speedUp()

                # avoid collision by either switching lane or slowing down
                if d <= car.v:
                    if cars.laneSwitchTrue(car, car.lane + 1):
                        cars.switchLane(car, car.lane + 1)
                        print(
                            f"car {car.c} changed from lane {car.lane} -> {car.lane + 1}"
                        )
                        lane_swap = True
                        cars.numLaneChanges += 1
                        assert cars.healthy()

                    elif cars.laneSwitchTrue(car, car.lane - 1):
                        cars.switchLane(car, car.lane - 1)
                        print(
                            f"car {car.c} changed from lane {car.lane} -> {car.lane - 1}"
                        )
                        cars.numLaneChanges += 1
                        lane_swap = True

                    else:
                        print(f"car {car.c} avoided collision v = {car.v} -> {d - 1}")
                        if d - 1 < 0:
                            print("error")
                        car.avoidCollision(d)

                # randomly slow down
                if (
                    car.v > 0 and (rng.rand() < self.p) and lane_swap is False
                ):  # Randomly slow down
                    print(f"car {car.c} slowed down v = {car.v} -> {car.v - 1}")
                    car.slowDown()

                # switch to inner lane if possible
                if (
                    cars.laneSwitchTrue(car, car.lane - 1)
                    and lane_swap is False
                    and cars.laneCount[car.lane - 1] / cars.numCars < 1 / cars.numLanes
                ):
                    print(f"car {car.c} changed from lane {car.lane} -> {car.lane - 1}")
                    cars.switchLane(car, car.lane - 1)
                    cars.numLaneChanges += 1

                # if the periodic distance becomes less than before (old_x > car.x), it has now become the last car
                old_x = car.x
                car.x = (car.x + car.v) % cars.roadLength
                if car == cars.first_cars[car.lane] and car.x < old_x:
                    cars.first_cars[car.lane] = car.prev
                    cars.last_cars[car.lane] = car
                assert cars.healthy()

                vSum += car.v

                car = car_next
        cars.t += 1
        print(f"Number of lane changes: {cars.numLaneChanges}")
        print(f"avg(v) = {vSum / cars.roadLength}")
        return vSum / cars.roadLength


############################################################################################


def _draw_cars(cars: Cars, cars_drawing):
    """
    Draw circular plotting.
    """
    theta = []
    r = []

    # Radier för körfälten
    lane_radii = {0: 0.6, 1: 0.9, 2: 1.2}
    positions = cars.getPositions()

    # Rita de sträckade linjerna för varje radie
    for radius in lane_radii.values():
        cars_drawing.plot(
            np.linspace(0, 2 * math.pi, 100),  # Vinklar (0 till 2π)
            [radius] * 100,  # Samma radie över hela cirkeln
            linestyle="--",  # Sträckad linje
            linewidth=0.5,  # Tunn linje
            color="gray",  # Grå färg
        )

    # Räkna ut bilarnas positioner och radier
    for idx in positions.keys():
        position = positions[idx][0]
        lane = positions[idx][1]
        # Konvertera position till radianer
        theta.append(position * 2 * math.pi / cars.roadLength)
        # Tilldela radie beroende på körfält
        r.append(lane_radii[lane])

    # Rita bilarna
    return cars_drawing.scatter(theta, r, c=cars.c, cmap="hsv")


def _draw_cars_2(cars, cars_drawing):
    positions = cars.getPositions()
    x = [positions[i][0] for i in positions.keys()]
    lane = [positions[i][1] for i in positions.keys()]
    colors = cars.c
    return cars_drawing.scatter(x, lane, c=colors, cmap="hsv")


def animate(framenr, cars, obs, propagator, road_drawing, stepsperframe):
    """Animation function which integrates a few steps and return a drawing"""

    for it in range(stepsperframe):
        # time.sleep(1.0)
        propagator.propagate(cars, obs)

    return (_draw_cars_2(cars, road_drawing),)


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
            positions = [pos[i][0] for pos in self.obs.positions]
            print(self.obs.time, positions)
            plt.scatter(self.obs.time, positions, label=f"Car {i}", s=10)
        plt.xlabel("Time")
        plt.ylabel("Position on road")
        plt.legend()
        plt.savefig(title + ".pdf")
        plt.show()

    #def getAverageFlowrate(self, propagator, numsteps=200):
    #    for it in range(numsteps):
    #        propagator.propagate(self.cars, self.obs)
    #
    #    return np.mean(self.obs.flowrate)

    #def getStdFlowrate(self, propagator, numsteps=200):
    #    for it in range(numsteps):
    #        propagator.propagate(self.cars, self.obs)

    #    return np.std(self.obs.flowrate[-50:])

    #def plotAvgFlowrate(
    #    self, attrName: str, attrValues: list, propagator: MyPropagator, numsteps: int
    #):
    #    x = attrValues
    #    y = []
    #    for value in attrValues:
    #        # cars = Cars(numCars=numCars, roadLength=roadLength, v0=v0, numLanes=numLanes)
    #
    #        for it in range(numsteps):
    #            propagator.propagate(self.cars, self.obs)
    #
    #        y.append(np.mean(self.obs.flowrate[-100:]))
    #
    #    plt.figure(figsize=(8, 6))
    #    plt.plot(x, y, marker="o", linestyle="-")
    #    plt.xlabel(attrName.capitalize())
    #    plt.ylabel("Average Flow rate")
    #    plt.title(f"Flow rate against {attrName}")
    #    plt.grid(True)
    #    plt.show()

    # Run without displaying any animation (fast)
    def run(
        self,
        propagator,
        numsteps=200,  # final time
        title="simulation",  # Name of output file and title shown at the top
    ):

        for it in range(numsteps):
            propagator.propagate(self.cars, self.obs)
            try:
                assert self.cars.healthy()
            except:
                print("error here")

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
        ax = fig.add_subplot(111)
        ax.axis("off")
        # Call the animator, blit=False means re-draw everything
        anim = animation.FuncAnimation(
            plt.gcf(),
            animate,  # init_func=init,
            fargs=(self.cars, self.obs, propagator, ax, stepsperframe),
            frames=numframes,
            interval=50,
            blit=True,
            repeat=False,
        )
        plt.show()
        self.plot_observables()
