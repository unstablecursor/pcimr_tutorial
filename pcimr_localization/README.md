# PCIMR Localization package
## Comments and  known problems
This package has logic to localize the robot by `/move` and `/scan` information. It uses Bayesian filter to find most probable robot location. This package complies with the basic standarts, tasks 1 ,2 and 3l; unfortunately task 4 is not complete. I plan to complete it *after* the deadline, so please check commit history for fair evaluation. \
I tried to do things using convolutions, instead of looping through grids multiple times, which I think would make everything faster for real world applications.\
 However, I did not understand how to use `/scan` message for probab. density estimation. I was expecting more information on that subject.\
 Side note: In task 3, robot may not reach the goal after some time due to very big increase in errors. \
 Launch the node by `roslaunch pcimr_localization localization.launch`
 