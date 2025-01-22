from unittest import TestCase
from traffic_template import *


class Test(TestCase):
    def test_Cars_init(self):
        cars = Cars()
        cars2 = Cars(numCars=8)
