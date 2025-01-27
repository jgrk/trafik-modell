import numpy as np

from traffic_template import *
from typing import Sequence


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

    def plotAvgFlowrate(
        self,
        attrName: str = None,
        attrValues: list[float] = None,
        attrName2: str = None,
        attrValues2: list[float] = None,
        numsteps: int = 200,
    ) -> None:
        if attrName is None:
            attrName = "carDensity"
            attrValues = [i / 10 for i in range(1, 11)]

        if attrName2 is None:
            attrName2 = ""
            attrValues2 = [1]

        plt.figure(figsize=(8, 6))
        for value2 in attrValues2:
            x = attrValues
            y = []
            for value in attrValues:
                cars = Cars(
                    numCars=int(value * self.roadLength),
                    roadLength=self.roadLength,
                    v0=self.v0,
                    numLanes=self.numLanes,
                    laneSwitchAllowed=self.laneSwitchAllowed,
                )
                prop = MyPropagator(vmax=self.vmax, p=self.p)
                if hasattr(cars, attrName):
                    setattr(cars, attrName, value)
                if hasattr(prop, attrName):
                    setattr(prop, attrName, value)
                if hasattr(cars, attrName2):
                    setattr(cars, attrName2, value2)
                if hasattr(prop, attrName2):
                    setattr(prop, attrName2, value2)

                sim = Simulation(cars=cars)
                sim.run(propagator=prop, numsteps=numsteps)
                y.append(np.mean(sim.obs.flowrate))

            plt.plot(x, y, marker="o", linestyle="-", label=f"{attrName2} = {value2}")
            plt.xlabel(attrName.capitalize())
            plt.ylabel("Average Flow rate")
            plt.title(f"Flow rate against {attrName}")
            plt.grid(True)
        plt.legend()
        plt.show()

    def getStatistics(
        self,
        nsims: int = 20,
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
            valueRange = [i for i in range(10, 200, 5)]
        plt.figure()
        for secondVals in secondArgVals:
            stdErrors = []
            for numSteps in valueRange:
                values = []
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
        plt.legend()
        plt.show()


def main():
    plotting = Plotting()
    # plotting.plotAvgFlowrate()
    plotting.getStatistics(secondArg="vmax", secondArgVals=[1, 3, 5])


if __name__ == "__main__":
    main()
