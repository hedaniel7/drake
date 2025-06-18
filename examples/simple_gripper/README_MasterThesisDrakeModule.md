This is the Master Thesis MIT Drake Simulation Code used for the simulation
of the grasps between a parallel jaw gripper (Robotiq 2f 140) and selected objects.

The main files are (please start reading the code by searching the string 'Part 1:' in these files):
- [robotiq_140_uogp_2024_force_application.cc](robotiq_140_uogp_2024_force_application.cc)
- [robotiq_140_uogp_2024.cc](robotiq_140_uogp_2024.cc)

and the object file folders in [uogp_2024](uogp_2024) (sidenote: 'uogp' stands for 
unknown objects grasp planner and was a project name used for my Master Thesis project):
- [Bowl](uogp_2024%2FBowl)
- [CheezItBox](uogp_2024%2FCheezItBox)
- [CoconutMilkCan](uogp_2024%2FCoconutMilkCan)
- [Duck](uogp_2024%2FDuck)
- [SodaCan](uogp_2024%2FSodaCan)
- [Sprayer](uogp_2024%2FSprayer)
- [TapeLyingDown](uogp_2024%2FTapeLyingDown)
- [WaterBottle](uogp_2024%2FWaterBottle)
- [WoodBlock](uogp_2024%2FWoodBlock)
- [YogaBall](uogp_2024%2FYogaBall)


Each object folder has a object mesh file and simulation settings detailing
object properties like weight and hydroelastic modulus used for the
simulation.


(Sidenote for shape completion:
The object meshes were predicted with the shape completion neural network
which estimates a complete mesh for the partial view point cloud of an objects.
The partial view point clouds are of objects recorded with real robotic
sensor data of DLR's AIMM.

More information about shape completion:
https://elib.dlr.de/195724/
)


To use these results here, one needs to [install drake](https://drake.mit.edu/installation.html) first

An example simulation binary call (after a bazel build of this folder) 
with some example flags (which could be automatically run by a script; 
That script then could for example automatically run and
evaluate multiple grasp poses predicted by the Contact-GraspNet neural network) would be:
- drake/examples/simple_gripper$ /home/dan/Projects/DrakeForks/MasterThesisDrakeModule/drake/bazel-bin/examples/simple_gripper/robotiq_140_uogp_2024_force_application --position=0.192292,0.079613,-0.016357 --orientation=-0.514892,-0.518741,0.615273,0.295354 --gripper_opening=0.048694 --manual_correction=0.0 --table_correction=-0.11498 --NoHeightCorrection --uogp_object=Sprayer --advanceSimTo=0.7 --SelectForceDirection=y --SelectMomentDirection=x --force_start=0.55 --force_end=0.56 --force_magnitude=180.5

That would rerun the DrakeDynamic results stored in a CSV (MasterThesisMethodsCode folder)
with additional settings set by Command line flags


More information about the Master Thesis can be found in this Video presentation:
https://www.youtube.com/watch?v=LdrG_YuUSac

