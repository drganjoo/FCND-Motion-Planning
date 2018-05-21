# FCND - 3D Motion Planning

![Quad Image](./misc/enroute.png)
![Video](./misc/flight.mp4)

## Ruberic Points

### Ruberic Point #1 (Explain Starter Code)

#### State Diagram

For this project, I have used my own implementation carried forward from the Backyard Flyer project. The major difference is in the way state diagrams have been implemented. All of the state diagram code resides in one function `create_state_diagram`. Each node of the state diagram is specified using `state_diagram.add` and the definition includes:

- State Name   
- Callback function to check tansition   
- Result of the callback -> Transition Function   

e.g.

```
state_diagram.add(States.PLANNING, MsgID.STATE, 
                        lambda: self.plan_status,
                        PlanResult.PLAN_SUCCESS, self.takeoff_transition,
                        PlanResult.PLAN_FAILED, self.disarming_transition)
```

The state name is `States.PLANNING`, callback function is `MsgId.STATE`, callback function is a lambda that checks if a plan has been found (PlanResult.PLAN_SUCCESS) then it will transit to `takeoff_transition` otherwise it will go to `disarming_tansition`.

#### Data Columns

Data read from the colliders.csv has the following columns:

| Column | Description|
|-|-|
|data[:, 0]| specifies the x (north) of the center of the obstacle|
|data[:, 1]| specifies the y (east) of the center of the obstacle|
|data[:, 2]| specifies the z coordinate of the center of the obstacle|
|data[:, 3]| is the half north side distance of the obstacle|
|data[:, 4]| is the half east side distance of the obstacle|
|data[:, 5]| is the half height of the obstacle|

Each data point is relative to the home location specified on the first line of the data file. In this particular case it is lat: 37.792480, lon: -122.397450

#### When is the plan created

When the drone is in the ARMED state, the code transits to PLANNING state (`planning_transition`). Following helper classes have been written to help in planning:

|Class Name  |Description|
|------------|-----------|
|GpsLocation |Represents Latitude, Longitude and Altitude (makes it easier not to confuse, lon and lat)|
|WorldMap |Loads data from colliders.csv, gets home location and then creates grid using Grid25 class|
|Grid25|A two dimensional np.array representation of the data in grid form. Each entry holds the height of the obstacle|
|Grid3d|A voxel based representation of the data|
|Planner|A*, Voronoi, KD Tree etc. are all used by the planner to plan a path from start to the goal state|

### Ruberic Point #2 (Reading Home Location)

WorldMap::load (planning_utils.py) reads the first line of the data file and then uses regular expression to read the latitude and longitude

```
with open(self.filename) as f:
    line = f.readline()
    m = re.match('lat0\s(.*),\slon0\s(.*)', line)
```

#### Start Location

In the `planning_transition` function, current global position is read using `self.global_position`, which in turn reads (self._longitude, self._latitude, self._altitude), then it is converted to local position using `global_to_local`


#### Goal Location

I used google maps to pin point the center of the map and then chose a goal location that is on top of a building [Google Map Link](https://goo.gl/maps/vAw9Hj2sjo82)


## Overall Planning Process

The basic idea was to use a 2d graph using Voronoi at a constant flight height. In case the goal state lied on the graph it would have been sufficient but if the goal state does not lie on the 2d graph then an action based plan is employed to use different altitude to reach the goal state.

The following is a detailed list of steps:

- Planner (defined in planner.py) object is created 
- `Planner::load_map` loads the data and the initial home position
- Global home is set using `set_home_position`
- Start and goal locations in local coordinate space are created

![Start and Goal](./misc/start_goal.png)

- Planner is asked to generate a route using `plan_route` (Planner.py line # 84)
* Planner create a 2d path through the grid using drone's minimum height (5m):
    - Creates **voronoi edges** using Voronoi.ridge_vertices. These are created using `Voronoi` and then using `bresenham` to exclude colliding edges

![Voronoi Edges](./misc/voronoi.png)

    - Creates a Graph of **voronoi edges**. It uses euclidean distance as the weight of edges (planner.py line # 172). A **KDTree of voronoi nodes** is created to find closest goal / start states

Nodes:

![Voronoi Graph Nodes](./misc/voronoi_nodes.png)

    - Find the **closest** node to the **start** state
    - Find the **closest** node to the **goal** state

*Green is the closest* node to the goal

![Closest](./misc/closest.png)

    - **A-Star is run** on the graph to find a path from the start to the closest goal state in 2d grid
    - Path returned is pruned using **colinearity checks**

![2dpath](./misc/2dpath.png)

    - If the closest node to goal is > 0.1m away, **it calls ActionPlanner** (defined in action_based_plan.py)
        - Action planner tries to find a path from the **original goal to the closest goal**
        - It uses **CubicAction**, which includes **changing altitude as part of action** in addition to changing the xy location
        - If an action plan can be found, it is added at the end of the planned route from voronoi

# Differences from Udacity Implementation

- Each node of the path also includes the cost of reaching the goal from that node
- Grid25 / Grid3d classes represent the grid
- Planner is a seperate class used for planning
- Wherever possible, instead of loops, direct np.array indices have been used

# Shortcomings

- Better culling alogrithm should be used to go through a more straighter route than the one followed.
- In case the drone starts off from a node that is not on the voronoi edges, the algorithm does not go to ActionPlanner directly and gets stuck
- When the closest goal state is looked for, there are times that the closest node returned from the Voronoi does not have a path from start to that location. This can be overcome by using a radius of closest points or maybe first by finding out a non obstructed closeby region and then using a point in there.
- My original plan was to create a receding horizon algorithm where by initial 2d path would have been computed and then a probabilistic roadmap would have been used. But due to shortage of time could not implement that.
