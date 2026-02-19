Fix:

- clean up

- Find out why ratio in sim is not right
        Its close. Arm looks right, ele does not
        
- Try to fix magcoder in sim

- LimeLight has been added. Need to check alot in code tho.

- Remove Smartdashboard for Subsystems before comp. Keep now for testing 

- Did not add every roller into sim, cause I am lazy.

- ADD SHIFT TIMER TO DASHBOARD!!! make sure it works correctly

- Maybe add either camera ignore or use gryo when going over bump

- Add manual overides. ie. Trust pose or not, trust zone or not, home

- Commands Fix:
        Try to fix Auto Sotm
        Maunal climb override Done
        Home all Done
        Pose off controls

- Teach Commands and add:
        Auto climb up, down and stow
        Climb set points
        Bump Shot
        Bump Pass
        Intake
        


Auto Naming Convention:

First Number is How many times it scores
        .5's means intake but not score
        C means climb at the end

First Letter is Start Location

S is for Shoot
SB is for Bump Shot
P is for Pass

Currently we are not planning to go over bump so LB/RB should not be used.


_____________________________________________________________________________
|                                            |                       |
|                                     L     LT                       |
|---|                                        |                       |
|   | Dep                                 ___|___                    |
|---|                       LS           |       |                   |
|                                        |  LB   |            LM     |
|                                        |       |                   |
|      LCLimb                            |_______|                   |
|------|                              CL | /   \ |                   |
|      |                    CS        C  | \   / |                   |
|      |                              CR |_______|                   |
|------|                                 |       |                   |
|      RCLimb                            |  RB   |             RM    |
|                                        |       |                   |
|                           RS           |       |                   |
||                                       |_______|                   |
|| Out                                       |                       |
||                                    R     RT                       |
||                                           |                       |
|____________________________________________|_______________________|________


Examples:
2C-R-Out-RS-RM-RS-RCLimb: Start Right, Intake at Outpost, Right Score, Intake from Right middle, Right Score, CLimb on right side
1C-CL-SB-LClimb: Start Center Left, Bump shot score, Climb on left side

        
------------------------------- Controller Layouts --------------------------------------

----------------------- Driver:

A:
B:
X:
Y:

Left Bumper:
Right Bumper:

Left Trigger:
Right Trigger:

Back Button:
Start Button:

Left Stick:
Right Stick:

D-Pad:

Left Back Button:
Right Back Button:

Rumble:

----------------------- Operator:

A:
B:
X:
Y:

Left Bumper:
Right Bumper:

Left Trigger:
Right Trigger:

Back Button:
Start Button:

Left Stick:
Right Stick:

D-Pad:

Left Back Button:
Right Back Button:

Rumble:













------------------------------- Furture Controller Layouts --------------------------------------

Driver:

A: Climb
B: Climb
X: Climb
Y: Climb

Left Bumper: Bump Score
Right Bumper: Score

Left Trigger: Close Intake
Right Trigger: Intake

Back Button: Enable Climb Overide
Start Button: Zero 

Left Stick: Drive
Right Stick: Rotate   If Climb Overide then manual movements

D-Pad:

Left Back Button:
Right Back Button: Stow

Rumble:

