import matplotlib.pyplot as plt
import numpy as np
import argparse
from typing import Iterable

from traffic_template import Cars, MyPropagator, Simulation
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
            p: float = 0.5,
            laneSwitchAllowed: bool = True,
    ):
        self.numCars = numCars
        self.roadLength = roadLength
        self.v0 = v0
        self.numLanes = numLanes
        self.p = p
        self.vmax = vmax
        self.laneSwitchAllowed = laneSwitchAllowed

    def plotFlowrateLane(self, numLanes: int = 3, nsteps: int = 300, fname='fundPlot.png'):

        plt.figure()
        for lane_idx in range(numLanes):
            n_lanes = lane_idx + 1
            flowrates = [0]
            carDensityList = [i / 50 for i in range(1, 50)]
            for carDensity in carDensityList:
                numCars = int(carDensity * self.roadLength * n_lanes)
                cars = Cars(
                    numCars=numCars,
                    roadLength=self.roadLength,
                    numLanes=n_lanes,
                    laneSwitchAllowed=self.laneSwitchAllowed,
                )
                prop = MyPropagator(vmax=self.vmax, p=self.p)
                sim = Simulation(cars=cars)
                sim.run(propagator=prop, numsteps=nsteps)
                flowrates.append(
                    np.mean(sim.obs.flowrate[-100:]) / (n_lanes))  # divide by n_lanes to get flowrate per unit area

            plt.plot(
                [0]+carDensityList,
                flowrates,
                linestyle="-",
                label=f"Number of lanes =  {lane_idx + 1}",
            )
        plt.legend()
        plt.xlabel(r"Car density, $\frac{N_{car}}{L*N_{lanes} }$")
        plt.ylabel(r"Mean flow rate, $\frac{v_{tot}}{L*N_{lanes}}$")
        plt.title(rf"Flow rate vs car density. Road length: $L= {self.roadLength} $")
        plt.grid(True)
        plt.savefig(fname=f"plots/{fname}", bbox_inches='tight')
        # plt.show()

    def fundPlot(self, nsteps: int = 100, showLaneChanges=True, fname:str = 'fundPlot.png'):
        """
        Plot the  fundamental diagram of the given simulation setting.
        To reproduce results from lab, run with numCars=10, roadLength=30, p=0.5

        :param nsteps: Number of simulation steps
        :type nsteps: int
        :param showLaneChanges: Set true to display number of lance changes (only for nlanes > 1)
        :type nsteps: bool

        :returns: None
        :rtype: None

        """
        plt.figure()
        laneChanges = []
        flowRates = [0]
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
            if showLaneChanges:
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
        plt.plot([0]+densities, flowRates, linestyle="-", label=f"Avg. flow rate")
        plt.legend()
        plt.xlabel(r"Car density, $\frac{N_{car}}{L*numLanes}$")
        plt.ylabel(r"Mean flow rate, $\frac{v_{tot}}{L*numLanes}$")
        plt.title(rf"Flow rate vs car density. Road length: $L= {self.roadLength} $")
        plt.grid(True)
        plt.savefig(fname=f"plots/{fname}", bbox_inches='tight')
        # plt.show()

    def getStatistics(
            self,
            valueRange: Iterable[float] = (10, 20, 30, 40, 50, 60, 70, 80, 90, 100),
            name_param_2: str = None,
            values_param_2: Iterable = None,
            fname:str = 'statError.png'
    ):
        """
        Get standard error of multiple flowrate simulations
        """
        local_attr = self.__dict__
        if name_param_2 is None:
            values_param_2 = [1]
        plt.figure()

        for secondVals in values_param_2:
            stdErrors = []
            for nsims in valueRange:
                values = []
                cars = Cars(
                    numCars=local_attr["numCars"],
                    roadLength=local_attr["roadLength"],
                    v0=local_attr["v0"],
                    numLanes=local_attr["numLanes"],
                    laneSwitchAllowed=local_attr["laneSwitchAllowed"],
                )
                prop = MyPropagator(vmax=local_attr["vmax"], p=local_attr["p"])
                sim = Simulation(cars=cars)
                for _ in range(nsims):
                    sim.reset(cars=cars)
                    sim.run(propagator=prop, numsteps=200)
                    values.append(np.mean(sim.obs.flowrate))
                stdErrors.append(np.std(values) / np.sqrt(nsims))

            plt.plot(
                valueRange,
                stdErrors,
                linestyle="-",
                label=f"{name_param_2} = {secondVals}",
            )
        plt.grid(True)
        plt.xlabel("Number of steps")
        plt.ylabel("Statistical Error")
        plt.title(
            f"Statistical Error between {nsims} simulations, against number of steps"
        )
        if name_param_2 is not None:
            plt.legend()
        plt.savefig(fname=f"plots/{fname}", bbox_inches='tight')
        # plt.show()

    def plotFlowrate(self, param_name: str = 'vmax', param_values: Iterable = (2, 3, 5, 10, 20, 40), fname='fundPlot_vmax.png'):
        """
        Plot the flowrate against a parameter in the simulation

        :param param_name: Name of parameter to vary
        :type param_name: str
        :param param_values: Values of parameter to vary
        :type param_values: iterable

        :returns: None
        :rtype: None

        """
        local_attr = self.__dict__
        plt.figure()

        for value in param_values:
            local_attr[param_name] = value
            flowrates = [0]
            densities = [i / 10 for i in range(1, 10)]
            for density in densities:
                cars = Cars(
                    numCars=int(density * local_attr['roadLength'] * local_attr['numLanes']),
                    roadLength=local_attr['roadLength'],
                    v0=local_attr['v0'],
                    numLanes=local_attr['numLanes'],
                    laneSwitchAllowed=local_attr['laneSwitchAllowed'],
                )
                prop = MyPropagator(vmax=local_attr['vmax'], p=local_attr['p'])
                sim = Simulation(cars=cars)
                sim.run(propagator=prop, numsteps=300)
                flowrates.append(np.mean(sim.obs.flowrate[-100:]) / local_attr['numLanes'])
            plt.plot(
                [0]+densities,
                flowrates,
                linestyle="-",
                label=f"{param_name} = {value}",
            )
        plt.legend()
        plt.xlabel(param_name)
        plt.ylabel("Flow rate")
        plt.title(f"Flow rate against {param_name}")
        plt.grid(True)
        plt.savefig(fname=f"plots/{fname}", bbox_inches='tight')
        # plt.show()


def main():
    """
    Main script for plotting

    Returns: None

    """
    plotting = Plotting()

    # Plot fundamental diagrams for different number of lanes in same plot, lane changing

    print("Plotting fundamental plot for all lanes... ")
    plotting.plotFlowrateLane(fname='fundPlot_all_lanes_w_lane_change.png')

    # Plot fundamental diagrams for different number of lanes in same plot, no lane changing applied

    noLaneChange = Plotting(laneSwitchAllowed=False)
    noLaneChange.plotFlowrateLane(fname='fundPlot_all_lanes_no_lane_change.png')

    print("Done!")
    # Fundamental plots for different speeds v = [2:40] (2 lanes)
    print("Plotting fundamental plot for different speeds... ")
    plotting = Plotting(numLanes=2)
    plotting.plotFlowrate(fname="fundPlot_vmax_2lanes.png")

    # Fundamental plots for different speeds v = [2:40] (3 lanes)

    plotting = Plotting(numLanes=3)
    plotting.plotFlowrate(fname="fundPlot_vmax_3lanes.png")

    print("Done!")

    # Fundamental plots for different road lengths (3 lanes, 2 lanes, 1 lane)
    print("Plotting fundamental plot for different road lengths... ")
    plotting = Plotting(numLanes=3)
    plotting.plotFlowrate(param_name='roadLength', param_values=(30, 50, 100), fname='fundPlot_roadLength_3_lanes.png')

    plotting = Plotting(numLanes=2)
    plotting.plotFlowrate(param_name='roadLength', param_values=(30, 50, 100), fname='fundPlot_roadLength_2_lanes.png')
    print("Done!")
    # Same for p
    print("Plotting fundamental plot for different p... ")
    plotting = Plotting(numLanes=3)
    plotting.plotFlowrate(param_name='p', param_values=(0.2, 0.5, 0.9), fname='fundPlot_p_3_lanes.png')

    plotting = Plotting(numLanes=2)
    plotting.plotFlowrate(param_name='p', param_values=(0.2, 0.5, 0.9), fname='fundPlot_p_2_lanes.png')
    print("Done!")
    # statistical error

    print("Plotting statistics... ")

    plotting = Plotting()
    plotting.getStatistics(fname='statError.png')
    plotting.getStatistics(name_param_2='numLanes', values_param_2=(1, 2, 3), fname='statError_numLanes.png')
    print("Done!")

    # lane changing behaviour for 3 lanes

    print("Plotting fundamental plot lane changes... ")
    plotting = Plotting(numLanes=3)
    plotting.fundPlot(fname="fundPlot_for_lane_changes.png", showLaneChanges=False)

    plotting = Plotting(numLanes=3, laneSwitchAllowed=False)
    plotting.fundPlot(fname="fundPlot_for_no_lane_changes.png", showLaneChanges=False)
    print("Done!")

    plotting = Plotting()
    plotting.plotFlowrate(param_name='laneSwitchAllowed', param_values=(True, False), fname='fundPlot_laneSwitch_added_and_removed.png')


if __name__ == "__main__":
    main()
