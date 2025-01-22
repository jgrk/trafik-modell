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

#from scipy.optimize import curve_fit
import math
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy.random as rng
import numpy as np
#from scipy.stats import sem



import matplotlib

class Cars:
    """Class for the state of a number of cars"""

    def __init__(self, numCars=5, roadLength=50, v0=1, numLanes=3):
        self.numCars = numCars
        self.roadLength = roadLength
        self.t = 0
        self.x = []
        self.v = []
        self.c = []
        self.lanes = []
        self.numLanes = numLanes  # Antalet körfält

        for i in range(numCars):
            # Placera bilar jämnt fördelade längs vägen
            self.x.append(i * (roadLength // numCars))
            self.v.append(v0)  # Hastighet
            self.c.append(i)  # Färger för visualisering
            # Tilldela körfält slumpmässigt
            self.lanes.append(rng.randint(1, numLanes + 1))


    # Function for computing x-distances
    def distance(self, i):
        # Calculate the periodic distance between car i and the car in front
        return (self.x[(i + 1) % self.numCars] - self.x[i]) % self.roadLength

    def laneOccupied(self, car_idx, lane_idx):
        for i in range(self.numCars):
            if ( self.lanes[i] == lane_idx ) and (self.x[i] == self.x[car_idx]):
                return True
        return False




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

    def __init__(self, vmax, p, safety_distance):
        super().__init__()
        self.vmax = vmax  # Maximal hastighet
        self.p = p  # Sannolikhet för att bromsa slumpmässigt
        self.safety_distance = safety_distance  # Minsta avstånd till bilen framför

    def timestep(self, cars, obs):
        for i in range(cars.numCars):
            # Beräkna avstånd till nästa bil
            d = cars.distance(i)

            # Omkörning: Kontrollera om bilen kan byta till vänster körfält
            if d <= self.safety_distance and cars.lanes[i] < cars.numLanes:
                if not cars.laneOccupied(car_idx=i, lane_idx=cars.lanes[i] + 1):
                    cars.lanes[i] += 1  # Byt till vänster körfält
                    continue

            # Återvänd till höger körfält om möjligt
            if cars.lanes[i] > 1 and not cars.laneOccupied(car_idx=i, lane_idx=cars.lanes[i] - 1):
                cars.lanes[i] -= 1

            # Acceleration: Öka hastigheten om det är säkert
            if d > cars.v[i] and cars.v[i] < self.vmax:
                cars.v[i] += 1

            # Deceleration: Sänk hastigheten om det är för trångt
            if d <= self.safety_distance:
                cars.v[i] = d - 1

            # Randomisering: Bromsa slumpmässigt
            if rng.rand() < self.p:
                cars.v[i] = max(0, cars.v[i] - 1)

            # Uppdatera position
            cars.x[i] = (cars.x[i] + cars.v[i]) % cars.roadLength

        # Uppdatera tid
        cars.t += 1

        # Beräkna flöde (antal bilar som passerar per tidsenhet)
        return sum(cars.v) / cars.roadLength



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

def main():
    cars = Cars(numCars=20, roadLength=100, v0=0.1, numLanes=3)
    simulation = Simulation(cars)
    propagator = MyPropagator(vmax=1, p=0.1, safety_distance=3)
    simulation.run_animate(propagator, numsteps=200)


if __name__ == "__main__":  
    main()


