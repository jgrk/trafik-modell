import matplotlib.pyplot as plt
import numpy as np
import argparse

from traffic_template import NewCar, Cars, MyPropagator, Simulation
from typing import Sequence

plt.rcParams["font.size"] = 14  # Ändra allmän textstorlek
plt.rcParams["axes.titlesize"] = 16  # Ändra storlek för titlar
plt.rcParams["axes.labelsize"] = 14  # Ändra storlek för axelns etiketter
plt.rcParams["xtick.labelsize"] = 12  # Ändra storlek för x-axelns ticks
plt.rcParams["ytick.labelsize"] = 12  # Ändra storlek för y-axelns ticks
plt.rcParams["legend.fontsize"] = 12  # Ändra storlek för legendtext
plt.rcParams["figure.titlesize"] = 18  # Ändra storlek för figurens titel


class Plotting:
    def __init__(
        self,
        numCars: int = 10,
        roadLength: int = 50,
        v0: int = 1,
        numLanes: int = 1,
        vmax: int = 2,
        p: float = 0.2,
        laneSwitchAllowed: bool = True,
    ):
        self.numCars = numCars
        self.roadLength = roadLength
        self.v0 = v0
        self.numLanes = numLanes
        self.p = p
        self.vmax = vmax
        self.laneSwitchAllowed = laneSwitchAllowed

    def plotFlowrateLane(self, numLanes: int = 3, nsteps: int = 300):

        plt.figure()
        for lane_idx in range(numLanes):
            n_lanes = lane_idx + 1
            flowrates = []
            carDensityList = [i / 50 for i in range(1, 50)]
            for carDensity in carDensityList:
                numCars = int(carDensity * self.roadLength * n_lanes)
                cars = Cars(
                    numCars=numCars,
                    roadLength=self.roadLength,
                    numLanes=lane_idx + 1,
                    laneSwitchAllowed=self.laneSwitchAllowed,
                )
                prop = MyPropagator(vmax=self.vmax, p=self.p)
                sim = Simulation(cars=cars)
                sim.run(propagator=prop, numsteps=nsteps)
                flowrates.append(np.mean(sim.obs.flowrate[-100:]) / n_lanes)

            plt.plot(
                carDensityList,
                flowrates,
                linestyle="-",
                label=f"Number of lanes =  {lane_idx + 1}",
            )
        plt.legend()
        plt.xlabel(r"Car density, $\frac{N_{car}}{L}$")
        plt.ylabel(r"Mean flow rate, $\frac{v_{tot}}{L}$")
        plt.title(rf"Flow rate vs car density. Road length: $L= {self.roadLength} $")
        plt.grid(True)
        plt.show()

    def fundPlot(self, nsteps: int = 100, showLaneChanges=True):
        plt.figure()
        laneChanges = []
        flowRates = []
        densities = [i / 50 for i in range(1, 50)]
        for carDensity in densities:
            numCars = int(carDensity * self.roadLength * self.numLanes)
            cars = Cars(
                numCars=numCars,
                roadLength=self.roadLength,
                numLanes=self.numLanes,
                laneSwitchAllowed=self.laneSwitchAllowed,
            )
            prop = MyPropagator(vmax=self.vmax, p=self.p)
            sim = Simulation(cars=cars)
            sim.run(propagator=prop, numsteps=nsteps)
            avgLaneChange = sum(sim.obs.numLaneChanges) / numCars
            laneChanges.append(avgLaneChange)
            flowRates.append(np.mean(sim.obs.flowrate[-100:]) / self.numLanes)
        if showLaneChanges:
            plt.plot(
                densities,
                laneChanges,
                linestyle="--",
                label=f"Avg. lane changes per car",
            )
        plt.plot(densities, flowRates, linestyle="-", label=f"Avg. flow rate")
        plt.legend()
        plt.xlabel(r"Car density, $\frac{N_{car}}{L*numLanes}$")
        plt.ylabel(r"Mean flow rate, $\frac{v_{tot}}{L*numLanes}$")
        plt.title(rf"Flow rate vs car density. Road length: $L= {self.roadLength} $")
        plt.grid(True)
        plt.show()

    def getStatistics(
        self,
        nsims: int = 10,
        valueRange: Sequence[float] = None,
        secondArg: str = None,
        secondArgVals: list[float] = None,
    ):
        """
        Get standard error of multiple flowrate simulations

        """
        if secondArg is None:
            secondArg = ""
            secondArgVals = [0]
        if valueRange is None:
            valueRange = [i for i in range(10, 300, 10)]
        plt.figure()
        for secondVals in secondArgVals:
            stdErrors = []
            for numSteps in valueRange:
                values = []
                if secondArg == "numLanes":
                    cars = Cars(
                        numCars=self.numCars,
                        roadLength=self.roadLength,
                        v0=self.v0,
                        numLanes=secondVals,
                        laneSwitchAllowed=self.laneSwitchAllowed,
                    )
                else:
                    cars = Cars(
                        numCars=self.numCars,
                        roadLength=self.roadLength,
                        v0=self.v0,
                        numLanes=self.numLanes,
                        laneSwitchAllowed=self.laneSwitchAllowed,
                    )
                prop = MyPropagator(vmax=self.vmax, p=self.p)
                if hasattr(cars, secondArg):
                    setattr(cars, secondArg, secondVals)
                if hasattr(prop, secondArg):
                    setattr(prop, secondArg, secondVals)

                sim = Simulation(cars=cars)
                for _ in range(nsims):
                    sim.reset(cars=cars)
                    sim.run(propagator=prop, numsteps=numSteps)
                    values.append(np.mean(sim.obs.flowrate))
                stdErrors.append(np.std(values) / np.sqrt(nsims))

            plt.plot(
                valueRange,
                stdErrors,
                linestyle="-",
                label=f"{secondArg} = {secondVals}",
            )
        plt.grid(True)
        plt.xlabel("Number of steps")
        plt.ylabel("Statistical Error")
        plt.title(
            f"Statistical Error between {nsims} simulations, against number of steps"
        )
        if secondArg != "":
            plt.legend()
        plt.show()

    def plotFlowrate(self, numsteps_list=None):
        if numsteps_list is None:
            numsteps_list = [100 + 5 * i for i in range(50)]
        std_vals = []
        cars = Cars(
            numCars=self.numCars,
            roadLength=self.roadLength,
            v0=self.v0,
            numLanes=self.numLanes,
            laneSwitchAllowed=self.laneSwitchAllowed,
        )
        prop = MyPropagator(vmax=self.vmax, p=self.p)
        sim = Simulation(cars=cars)
        for numsteps in numsteps_list:
            sim.reset(cars=cars)
            std_vals.append(sim.getStdFlowrate(numsteps=numsteps, propagator=prop))
        plt.figure(figsize=(8, 6))
        plt.plot(numsteps_list, std_vals, linestyle="-")
        plt.xlabel("Number of steps")
        plt.ylabel("Deviation")
        plt.title(f"Standard deviation in flowrate against number of steps")
        plt.show()


def main():
    plotting = Plotting(
        roadLength=50, numLanes=3, numCars=10, vmax=5, laneSwitchAllowed=True, p=0.5
    )
    # plotting.plotAvgFlowrate(attrName2='vmax', attrValues2=[2, 3, 5, 10, 15, 40])
    # plotting.plotAvgFlowrate(showLaneChanges=True, attrName2='numLanes', attrValues2=[1, 2, 3])
    # plotting.getStatistics(secondArg='numLanes', secondArgVals=[1,2,3])
    # plotting.plotAvgFlowrate(attrName2='numLanes')
    ##plotting.getStatistics(secondArg="vmax", secondArgVals=[1, 3, 5])
    # plotting.plotFlowrate()
    # plotting.plotFlowrateLane(numLanes=3)
    # plotting.plotAvgFlowrate(attrName2='vmax', attrValues2=[2, 3, 5, 8, 10, 15, 25, 40])
    plotting.plotFlowrateLane(nsteps=200)
    # plotting.fundPlot()


if __name__ == "__main__":
    main()
