import argparse
from unittest import TestCase
from traffic_template import *

parser = argparse.ArgumentParser(description="Traffic Template Tester")
parser.add_argument("--numCars", default=20, type=int)
parser.add_argument("--roadLength", default=30, type=int)
parser.add_argument("--v0", default=1, type=int)
parser.add_argument("--numLanes", default=2, type=int)
parser.add_argument("--vmax", default=2, type=int)
parser.add_argument("--p", default=0.5, type=float)
parser.add_argument("--numSteps", default=500, type=int)
parser.add_argument("--laneSwitching", default=True, type=bool)
# parser.add_argument("--switch", default=0, type=int)
args = parser.parse_args()

cars = Cars(
    numCars=args.numCars, roadLength=args.roadLength, v0=args.v0, numLanes=args.numLanes,
    laneSwitchAllowed=args.laneSwitching

)
sim = Simulation(cars=cars)
prop = MyPropagator(vmax=args.vmax, p=args.p)
sim.run(propagator=prop, numsteps=args.numSteps)

sim.run_animate(prop)
# sim.plotAvgFlowrate(attrName='carDensity', attrValues = [i/10 for i in range(1, 11)], propagator=prop, numsteps=200 )


class Test(TestCase):
    def test_Cars_init(self):
        cars = Cars()
        cars = Cars(numCars=8, numLanes=3)

        for lane_idx in range(cars.numLanes):
            car = cars.last_cars[lane_idx]

            while car != cars.first_cars[lane_idx]:
                self.assertGreater(car.next.x, car.x)
                self.assertEqual(car.lane, lane_idx)
                car = car.next

    def test_laneSwitch(self):
        cars = Cars(numCars=8, numLanes=3)
        cars.first_cars[2] = cars.last_cars[2] = None
        for lane_idx in range(cars.numLanes):
            car = cars.last_cars[lane_idx]

    def test_simulation_init(self):
        cars = Cars()
        sim = Simulation()
        prop = MyPropagator(vmax=5, p=0.7)
        sim.run(prop)

    def test_animate_init(self):
        sim = Simulation()
        prop = MyPropagator(vmax=5, p=0.7)
        sim.run_animate(prop)
