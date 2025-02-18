<a id="plotting"></a>

# plotting

<a id="plotting.Plotting"></a>

## Plotting Objects

```python
class Plotting()
```

<a id="plotting.Plotting.getStatistics"></a>

#### getStatistics

```python
def getStatistics(nsims: int = 10,
                  valueRange: Sequence[float] = None,
                  secondArg: str = None,
                  secondArgVals: list[float] = None)
```

Get standard error of multiple flowrate simulations

<a id="traffic_template"></a>

# traffic\_template

<a id="traffic_template.NewCar"></a>

## NewCar Objects

```python
class NewCar()
```

<a id="traffic_template.NewCar.speedUp"></a>

#### speedUp

```python
def speedUp()
```

Increase velocity by one unit


<a id="traffic_template.NewCar.slowDown"></a>

#### slowDown

```python
def slowDown()
```

Decrease velocity by one unit


<a id="traffic_template.NewCar.avoidCollision"></a>

#### avoidCollision

```python
def avoidCollision(d: int)
```

Collision avoidance

**Arguments**:

- `d` (`int:`): Distance to car ahead

**Returns**:

`None`: None

<a id="traffic_template.Cars"></a>

## Cars Objects

```python
class Cars()
```

Groups a set of NewCar objects and sets simulation parameters

Includes functionality for each car when considered in traffic

Used together with Simulation class as a parameter.

**Example**:

  >>> cars = Cars()
  >>> sim = Simulation(cars=cars)

<a id="traffic_template.Cars.laneSwitchTrue"></a>

#### laneSwitchTrue

```python
def laneSwitchTrue(car: NewCar, lane_num: int) -> bool
```

Check if lane switch is possible

**Arguments**:

- `car` (`NewCar`): NewCar object
- `lane_num` (`int`): Target lane

**Returns**:

`bool`: True if lane switch is possible

<a id="traffic_template.Cars.switchLane"></a>

#### switchLane

```python
def switchLane(car: NewCar, lane_num: int)
```

Perform lane switch on car

**Arguments**:

- `car` (`NewCar`): NewCar object
- `lane_num` (`int`): Target lane

**Returns**:

`None`: None

<a id="traffic_template.Observables"></a>

## Observables Objects

```python
class Observables()
```

Class for storing observables

<a id="traffic_template.BasePropagator"></a>

## BasePropagator Objects

```python
class BasePropagator()
```

<a id="traffic_template.BasePropagator.propagate"></a>

#### propagate

```python
def propagate(cars, obs)
```

Perform a single integration step

<a id="traffic_template.BasePropagator.timestep"></a>

#### timestep

```python
def timestep(cars, obs)
```

Virtual method: implemented by the child classes

<a id="traffic_template.ConstantPropagator"></a>

## ConstantPropagator Objects

```python
class ConstantPropagator(BasePropagator)
```

Cars do not interact: each position is just
updated using the corresponding velocity

<a id="traffic_template.MyPropagator"></a>

## MyPropagator Objects

```python
class MyPropagator(BasePropagator)
```

Propagator with lane switching logic

Used together with Cars and Simulation.

**Example**:

  >>> cars = Cars()
  >>> sim = Simulation(cars=cars)
  >>> prop = MyPropagator()
  >>> sim.run(propagator=prop)

<a id="traffic_template.animate"></a>

#### animate

```python
def animate(framenr, cars, obs, propagator, road_drawing, stepsperframe)
```

Animation function which integrates a few steps and return a drawing

<a id="script"></a>

# script

<a id="testing"></a>

# testing

