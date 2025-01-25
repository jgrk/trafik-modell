from unittest import TestCase
from traffic_template import *

cars = Cars(numCars=10, roadLength=100, v0=1, numLanes=3)
sim = Simulation(cars=cars)
prop = MyPropagator(vmax=5, p=0.2)
# sim.run(propagator=prop)
sim.run_animate(prop)


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
            # print(cars.laneSwitchTrue(car, car.lane + 1))

    def test_simulation_init(self):
        cars = Cars()
        sim = Simulation()
        prop = MyPropagator(vmax=5, p=0.7)
        sim.run(prop)

    def test_animate_init(self):
        sim = Simulation()
        prop = MyPropagator(vmax=5, p=0.7)
        sim.run_animate(prop)
