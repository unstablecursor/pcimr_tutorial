# PCIMR Localization package
## Comments and  known problems
This package has logic to localize the robot by `/move` and `/scan` information. It uses Bayesian filter to find most probable robot location. This package complies with the basic standarts, tasks 1 and 2, however for tasks 3 and 4 it is not complete. I plan to complete them *after* the deadline, so please check commit history for fair evaluation. 
I tried to do things using convolutions, instead of looping through grids multiple times, which I think would make everything faster for real world applications.
 However, I did not understand how to use `/scan` message for probab. density estimation. I was expecting more information on that subject.
 