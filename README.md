# SwarmFuzz

SwarmFuzz is a fuzzing framework to efficiently find SPVs (Swarm Propagation Vulnerabilities) in drone swarms. It uses a combination of graph theory and gradient-guided optimization to find the potential attack parameters. 

For more details about SwarmFuzz, please refer to the paper published in DSN 2023 "SwarmFuzz: Discovering GPS Spoofing Attacks in Drone Swarms".


## Get started

### 1) Requirement
Matlab R2022b with the Statistics and Machine Learning Toolbox. 

### 2) Clone the repository
We use [Swarmlab](https://github.com/lis-epfl/swarmlab) as the simulator for drone swarms.
```
git clone https://github.com/lis-epfl/swarmlab.git
git clone https://github.com/DependableSystemsLab/SwarmFuzz.git
```
Suppose that the Swarmlab repository is under the path `swarmlab`, and the SwarmFuzz repository in under the path `SwarmFuzz`.


### 3) Change the Swarmlab accordingly
We have to copy SwarmFuzz into the Swarmlab folder and also change some settings in Swarmlab to record the swarm's state.

Run the following commands
```
cp -r SwarmFuzz/fuzz/ swarmlab
cp SwarmFuzz/changed_swarmlab/compute_vel_vasarhelyi.m swarmlab/@Swarm
cp SwarmFuzz/changed_swarmlab/param_swarm.m swarmlab/parameters/
cp SwarmFuzz/changed_swarmlab/example_vasarhelyi.m swarmlab/examples/examples_swarm/
cp SwarmFuzz/changed_swarmlab/Swarm.m swarmlab/@Swarm
cp SwarmFuzz/changed_swarmlab/create_shifted_buildings.m swarmlab/graphics/graphics_map/
```
### 4) Open Swarmlab
Under `swarmlab/`, open MATLAB, 

```
cd swarmlab
matlab
```
### 5) Run SwarmFuzz
In the command window in MATLAB, run the following command
```
cd fuzz/
swarmfuzz(200, 201, 5, 5) 
```

The arguments for the `fuzz` function are `(seedStart, seedEnd, dev, nb)`
- `seedStart`: the seed for the first mission 
- `seedEnd`: the seed for the last mission
- `dev`: GPS spoofing deviation
- `nb`: number of drones in the swarm
Therefore, by running `fuzz(200, 201, 5, 5)`, we are fuzzing the 5-drone swarm missions with seed 200 and 201, with the 5m GPS spoofing deviation.

### 6) Interprete the result
- `fuzz/seedpools/pool{seed}.csv`: Generated seedpool for each mission. Each row represents `[target_id, victim_id, deviation_direction, spoofing_time]`. The number of rows represents the number of potential attack-victim drone pairs in this mission.
- `fuzz/attResults/att_results{seed}.csv`: Attack results found by SwarmFuzz for each mission. Each row represents `[collision_or_not, init_VDO, current_VDO, spoofing_start_time, spooing_duration, attack_id, victim_id, spooing_deviation]`. The number of rows present the attack results for each seed.
- `fuzz/search/iteration{seed}.csv`: Results for each gradient descent search iteration. The number of rows represents the overhead of the fuzzing, i.e., the number of iterations taken to find the attack.  


## Configurations
- The default number of drones in the swarm is 5, if you want to test the swarm of 10/15 drones, do the following:
    - Change the `nb` argument according when calling fuzz(seedStart, seedEnd, dev, nb)
    - In the file `parameters/param_swarm.m`, change the variable `p_swarm.nb_agents`.
    - In the file `@Swarm/Swarm.m`, in the function `get_colors(self)`, change `colors` to be an array with 10/15 rows. You can specify the color as you like.

## Citations
TODO

## Contact
For questions about our paper or this code, please open an issue or contact Elaine Yao (elainedv111@gmail.com)