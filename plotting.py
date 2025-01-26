from traffic_template import *


class Plotting:
    def __init__(self, numCars: int = 10, roadLength: int = 80, v0: int = 1, numLanes: int = 3,
                 carDensity: float | None = None, vmax: int = 5, p: float = 0.2, laneSwitchAllowed: bool = True):
        if carDensity is not None:
            self.carDensity = carDensity
            self.numCars = carDensity * roadLength
        else:
            self.numCars = numCars
        self.roadLength = roadLength
        self.v0 = v0
        self.numLanes = numLanes
        self.p = p
        self.vmax = vmax
        self.laneSwitchAllowed = laneSwitchAllowed

    def plotAvgFlowrate(self, attrName: str, attrValues: list[float], numsteps: int = 200) -> None:
        x = attrValues
        y = []
        for value in attrValues:
            if attrName == 'carDensity':
                cars = Cars(numCars=self.numCars, roadLength=self.roadLength, v0=self.v0, numLanes=self.numLanes,
                            carDensity=value, laneSwitchAllowed=self.laneSwitchAllowed)
            else:
                cars = Cars(numCars=self.numCars, roadLength=self.roadLength, v0=self.v0, numLanes=self.numLanes,
                            carDensity=self.carDensity, laneSwitchAllowed=self.laneSwitchAllowed)
            prop = MyPropagator(vmax=self.vmax, p=self.p)
            if hasattr(cars, attrName):
                setattr(cars, attrName, value)
            if hasattr(prop, attrName):
                setattr(prop, attrName, value)
            sim = Simulation(cars=cars)
            sim.run(propagator=prop, numsteps=numsteps)
            y.append(np.mean(sim.obs.flowrate))

        plt.figure(figsize=(8, 6))
        plt.plot(x, y, marker='o', linestyle='-')
        plt.xlabel(attrName.capitalize())
        plt.ylabel('Average Flow rate')
        plt.title(f'Flow rate against {attrName}')
        plt.grid(True)
        plt.show()


def main():
    plotting = Plotting(carDensity=1, laneSwitchAllowed=True)
    plotting.plotAvgFlowrate('carDensity', [i / 10 for i in range(1, 10)])


if __name__ == '__main__': main()