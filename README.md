Overview
========

The goal of the project is to safely navigate the ego car on the highway
in the simulator. The highway has three lanes and other traffic is
driving at a max speed of 50 PMH. The 50 MPH is the speed limit. The
path planner receives the sensor fusion data from the simulator. The
sensor fusion data contain location and speed information from
surrounding traffic. Map data is available in the form of waypoints. The
car needs to ensure to keep on the road and for most of the time within
a lane (only for lane change the car should be outside of a lane). The
car needs to drive safely and should not crash into other cars. It is
not sufficient to stay only on the initial lane – the car should perform
lane changes to drive as fast as possible but needs to respect the speed
limit. Also, the car should not experience total acceleration over 10
m/s\^2 and jerk that is greater than 10 m/s\^3

Reflection
==========

The implementation of the path planner work pretty well. However, it
doesn’t yet consider misbehaviour of other vehicles which could get to
close to the ego vehicle.

For future enhancements of the code I would continue the modularization
into Behavior planner and trajestory planner. Especially, the
implementation of a statechart for behavior planning and the evaluation
of different behaviours and trajestories based on cost functions would
improve the code of the path planner.

Write-up Behavior Planning

1.  Got he car moving within the current lane

2.  Ensure a smooth start

3.  Implemented Spline based trajectory planning for follow other car
    scenario. Incorporated last points of old path to ensure that we are
    not create jerk between the two pathes

4.  Implemented Polynominal trajectory planning for lane change. Again
    incorporate previous path points as a starting point for the
    trajectory. No lane change implemented.

5.  Up to here the ego crashes into other cars as they are incorporated
    in any behavior planning

6.  Prediction

    a.  Implementing prediction. Out of all sensor fusion observation we
        identify per lane the cars which are within the senor range
        directly behind and in front of us. This gives us for each
        lane (3) a back\_car and a front\_car, in total 6 cars.

    b.  For those 6 cars we calculate their trajectories within the
        planning horizon

    c.  For the egoCar we create 3 trajectories for each adjacent lanes
        each with a different time horizon (d will be set to middle of
        the lane)

    d.  The remaining egoCar trajectories are provided as input to the
        behavior planner

7.  Behavior planning

    a.  The behavior planner uses a FSM to evaluate potential target
        actions based on the current state. Need to understand how to
        deal with the “prepare lane change state”

        ![Finite state machine diagram for a self-driving car path
        planner](media/image1.png){width="4.101449037620298in"
        height="2.1194903762029744in"}

    b.  For each of the egoCar trajectories we first check for collision
        with other cars on other lanes (for our lane we will later check
        if we need to reduce speed to prevent a collision). The
        collision check is based on the [SAT (Separating Axis
        Theorem).](http://www.dyn4j.org/2010/01/sat/) This process
        assumes that the other vehicles are moving with a constant
        velocity. To ensure safety and a large enough gap for a lane
        change, we assume a slightly higher speed if the vehicle is
        behind us and a slightly slower speed if the vehicle is in front
        of us (for those vehicles on other than our lane). This creates
        a large enough gap for a lane change. We eliminate the
        trajectories which will lead to a collision (no cost function is
        used here)

    c.  The behavior planner applies a cost function to the trajectories
        to identify the one with the lowest cost

    d.  Candidates for the cost function:

        i.  Jerk minimizing

        ii. speed

        iii. distance to car ahead

        iv. 

![](media/image2.png){width="4.639045275590552in"
height="3.794233377077865in"}

![](media/image3.png){width="5.414505686789151in"
height="2.1368963254593174in"}

![](media/image4.png){width="6.295833333333333in"
height="3.5430555555555556in"}

![](media/image5.png){width="2.7364599737532807in"
height="2.884058398950131in"}

![](media/image6.png){width="4.976201881014873in"
height="1.5873786089238846in"}

![](media/image7.png){width="5.115941601049869in"
height="1.603740157480315in"}

  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  **State**             **Action**                                                                                                                                                                                              **Prerequisites for Trigger**                                                                 **Trigger**
  --------------------- ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- --------------------------------------------------------------------------------------------- ---------------
  Keep Lane             if EgoCar\_speed &lt; speed\_limit or CarAhead\_speed)                                                                                                                                                                                                                                                
                                                                                                                                                                                                                                                                                                                              
                        -   accelerate                                                                                                                                                                                                                                                                                        
                                                                                                                                                                                                                                                                                                                              

                        if distance between EgoCar and CarAhead &lt; SafetyDist set EgoCar\_speed to CarAhead\_speed                                                                                                                                                                                                          
                                                                                                                                                                                                                                                                                                                              
                        Create trajectory                                                                                                                                                                                                                                                                                     

  Prepare Lane Change                                                                                                                                                                                                           CarAhead\_speed &lt; speed\_limit and distance between EgoCar and CarAhead &lt;= SafetyDist   SlowCarAhead

  Prepare Lane Change   If in left\_lane: Create trajectory for lange change to middle lane and trajectory for keeping lane                                                                                                                                                                                                   
                                                                                                                                                                                                                                                                                                                              
                        Apply cost functions                                                                                                                                                                                                                                                                                  
                                                                                                                                                                                                                                                                                                                              
                        Select trajectory with best cost                                                                                                                                                                                                                                                                      
                                                                                                                                                                                                                                                                                                                              
                        In case keeping lane has better cost TRIGGER LangeChangeNotDesireable                                                                                                                                                                                                                                 
                                                                                                                                                                                                                                                                                                                              
                        Else TRIGGER ChangeToRight                                                                                                                                                                                                                                                                            
                                                                                                                                                                                                                                                                                                                              
                        If in right\_lane: Create trajectory for lange change to middle lane and trajectory for keeping lane                                                                                                                                                                                                  
                                                                                                                                                                                                                                                                                                                              
                        Apply cost functions                                                                                                                                                                                                                                                                                  
                                                                                                                                                                                                                                                                                                                              
                        Select trajectory with best cost                                                                                                                                                                                                                                                                      
                                                                                                                                                                                                                                                                                                                              
                        In case keeping lane has better cost TRIGGER LangeChangeNotDesireable                                                                                                                                                                                                                                 
                                                                                                                                                                                                                                                                                                                              
                        Else TRIGGER ChangeToLeft                                                                                                                                                                                                                                                                             
                                                                                                                                                                                                                                                                                                                              
                        If in middle\_lane: Create trajectories for change to left, change to right and stay in the lane. Apply cost functions                                                                                                                                                                                
                                                                                                                                                                                                                                                                                                                              
                        Select trajectory with best cost                                                                                                                                                                                                                                                                      
                                                                                                                                                                                                                                                                                                                              
                        Cost keep lane best: TRIGGER LangeChangeNotDesireable                                                                                                                                                                                                                                                 
                                                                                                                                                                                                                                                                                                                              
                        Cost right lane best: TRIGGER ChangeToRight                                                                                                                                                                                                                                                           
                                                                                                                                                                                                                                                                                                                              
                        Cost left lane best: TRIGGER ChangeToLeft                                                                                                                                                                                                                                                             

  Lane Change Right     If lane change not in progress calculate trajectory for lane change else check if lane change finished (we assume that lane change are executed within 1 sec. If finished TRIGGER LaneChangeCompleted                                                                                                 ChangeToRight

  Lane Change Left      If lane change not in progress calculate trajectory for lane change else check if lane change finished (we assume that lane change are executed within 1 sec. If finished TRIGGER LaneChangeCompleted                                                                                                 ChangeToLeft

                                                                                                                                                                                                                                                                                                                              

                                                                                                                                                                                                                                                                                                                              

                                                                                                                                                                                                                                                                                                                              

                                                                                                                                                                                                                                                                                                                              
  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

![](media/image8.png){width="6.295833333333333in"
height="5.048611111111111in"}

Overview
========

The goal of this project is to safely navigate around a virtual highway
with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
The path planner gets the localization of the car and sensor fusion data
from a simulator. A map is available that lists the waypoints around the
highway. The car should try to go as close as possible to the 50 MPH
speed limit, which means passing slower traffic when possible. The car
should avoid hitting other cars at all cost as well as driving inside of
the marked road lanes at all times, unless going from one lane to
another. The car should be able to make one complete loop around the
6946m highway. Since the car is trying to go 50 MPH, it should take a
little over 5 minutes to complete 1 loop. Also the car should not
experience total acceleration over 10 m/s\^2 and jerk that is greater
than 10 m/s\^3.

The following image shows successful navigation around the virtual
highway for more that 40
miles. ![Driving](media/image9.png){width="6.295833333333333in"
height="4.999305555555556in"}

Safe lane changes are implemented as shown in the video. ![White
Line](media/image10.jpeg){width="6.295833333333333in"
height="4.722222222222222in"}

Files
=====

The project consists of the following files:

-   [README.md](https://github.com/MarkBroerkens/CarND-Path-Planning-Project/blob/master/README.md) (writeup
    report) documentation of the results

-   [main.cpp](https://github.com/MarkBroerkens/CarND-Path-Planning-Project/blob/master/src/main.cpp) The
    main c++ file that implements the communication with the simulator.

-   [path\_planner.cpp](https://github.com/MarkBroerkens/CarND-Path-Planning-Project/blob/master/src/path_planner.cpp) The
    actual implementation of the path planner and the generation of the
    trajestories.

-   [vehicle.cpp](https://github.com/MarkBroerkens/CarND-Path-Planning-Project/blob/master/src/vehicle.cpp) Class
    that represents the ego vehicle or other vehicle that are detected
    by sensor fusion.

-   [util.cpp](https://github.com/MarkBroerkens/CarND-Path-Planning-Project/blob/master/src/util.cpp) Utility
    functions

Description of the Path Planner
===============================

The path planner is initialized by the main.cpp with the map of the
highway. Based on the data from the simulator, the path planner creates
an instance of the ego vehicle and the surrounding vehicles.

Behavior
--------

The default behavior of the ego vehicle is to stay in its lane with the
maximum speed (49.5 miles per hour). In case the prediction detects that
the ego vehicle will get to close to a vehicle that is driving in front
of the vehicle in the same lane, it has the following options (see
PathPlanner::path()):

-   slow down: if there is no faster lane or if it is not safe to change
    the lane

-   change lane: if a faster lane is detected and it is safe to change
    the lane

### Safety distance

The safe distance to a car that is driving in front of the ego vehicle
is calculated in PathPlanner::safetyDistance based on the speed of the
ego vehicle:

**double** PathPlanner**::**safetyDistance(**double** speed\_mps) {

*// see
http://www.softschools.com/formulas/physics/stopping\_distance\_formula/89/*

**double** reaction\_distance **=** speed\_mps **\*** REACTION\_TIME\_S;

**double** brake\_distance **=** (speed\_mps **\*** speed\_mps)

**/** (2 **\*** CAR\_COEFFICIENT\_OF\_FRICION **\***
CAR\_ACCELERATION\_DUE\_TO\_GRAVITY\_MPS2);

**return** reaction\_distance **+** brake\_distance;

}

### Prediction

The prediction of the location of the ego vehicle is defined by the end
of the previous path that was calculated in the previous iteration.

The prediction of the location of other vehicles that are driving in
front of the vehicle is calcluated according to the follwing expression:

check\_car\_s **+=** ((**double**) prev\_size **\*** TICK\_S **\***
check\_speed);

### Identification of the fastest lane

For each lane the minimum lane speed is calculated based on the cars
that are visible in the range of the front sensor or up to 10 meters
behind the ego vehicle. This helps avoiding collision with vehicles that
are directly next to the ego vehicle.

**double** PathPlanner**::**laneSpeed(**int** lane) {

**double** lane\_speed **=**
milesPerHourToMetersPerSecond(MAX\_SPEED\_MPH);

**for** (Vehicle vehicle **:** vehicles) {

**if** (vehicle.lane **==** lane) {

**double** distance\_to\_vehicle\_ahead **=** wrappedDistance(ego.s,
vehicle.s);

**double** distance\_to\_vehicle\_behind **=**
wrappedDistance(vehicle.s, ego.s);

**if** (((distance\_to\_vehicle\_ahead **&lt;** FRONT\_SENSOR\_RANGE\_M)
**||** (distance\_to\_vehicle\_behind **&lt;** 10))

**&&** (vehicle.speed **&lt;** lane\_speed)) {

lane\_speed **=** vehicle.speed;

}

}

}

**return** lane\_speed;

}

In order to select the fastest lane, we iterate over all lanes and
select the fastest lane. If several lanes allow the same fastest speed,
the lane that is closest to the ego vehicle is chosen. This helps
avoiding unnecessary lane changes.

**int** PathPlanner**::**fastestLane() {

**int** fastest\_lane **=** ego.lane;

**double** fastest\_speed **=** laneSpeed(fastest\_lane);

**for** (**int** lane **=** 0; lane **&lt;** NUMBER\_OF\_LANES;
lane**++**) {

**double** lane\_speed **=** laneSpeed(lane);

**if** ((lane\_speed **&gt;** fastest\_speed)

**||** ((lane\_speed **==** fastest\_speed)

**&&** (fabs(lane **-** ego.lane) **&lt;** fabs(fastest\_lane **-**
ego.lane)))) {

fastest\_speed **=** lane\_speed;

fastest\_lane **=** lane;

}

}

**return** fastest\_lane;

}

### Safety

The behavior planner makes sure that the ego vehicle stays in its lane
if a lane change is not safe. A lane change is considered safe if there
is no other vehicle in the target lane within the safety distance of the
ego vehicle in front and the safety distance of any vehicle in the
target lane.

**double** PathPlanner**::**safetyCosts(**int** lane) {

*// find vehicles in the lane that might cause trouble*

*// cars in the safety distance before or behind us*

**double** safety\_costs **=** 0.0;

**for** (Vehicle vehicle **:** vehicles) {

**if** (vehicle.lane **==** lane) {

**if** ((wrappedDistance(vehicle.s, ego.s) **&lt;**
safetyDistance(vehicle.speed)) */\* ego in front of vehicle \*/*

**||** (wrappedDistance(ego.s, vehicle.s) **&lt;**
safetyDistance(ego.speed)) */\* ego vehicle in front of ego \*/*) {

safety\_costs **+=** 1.0;

}

}

}

**return** safety\_costs;

}

### Calculation of distance when a new lap of the highway is started

The simulator restarts the driven distance of the track after
TRACK\_LENGTH\_M meters. A helper function helps to calculate the
distance even if the vehicles enter a new lap:

**double** PathPlanner**::**wrappedDistance(**double** back\_s,
**double** front\_s) {

**double** distance **=** (front\_s **-** back\_s **+**
TRACK\_LENGTH\_M) **-** TRACK\_LENGTH\_M;

**if** (distance **&lt;** 0) {

distance **=** TRACK\_LENGTH\_M **+** distance;

}

**return** distance;

}

### Trajestory

A smooth trajestory is calculated using a spline that contains some
previous path points of the ego vehicle and some future points from the
map. The actual future path points of the ego vehicle are derived from
the spline. This helps to avoid jerk.

In order to avoid abrupt changes of the velocity, we incrementally
increase or decrease the distance between the points of the path.

Reflection
==========

The implementation of the path planner work pretty well. However, it
doesn’t yet consider misbehaviour of other vehicles which could get to
close to the ego vehicle.

For future enhancements of the code I would continue the modularization
into Behavior planner and trajestory planner. Especially, the
implementation of a statechart for behavior planning and the evaluation
of different behaviours and trajestories based on cost functions would
improve the code of the path planner.
