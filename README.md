# CarpusDiem: Equine Carpus Lameness Diagnostic System with MATLAB
CarpusDiem is an interactive MATLAB script designed to aid carpal lameness diagnostic testing for all experience levels of equine management.
 
CarpusDiem utilizes the Shi-Tomasi Corner Detection and a minimum eigenvalue algorithm to detect trackable points from user input. Points are continuously filtered against a forward-backward error threshold to maintain validity. Valid points are used to calculate velocity, acceleration, force of movement, and final net force acting on the carpus as a gait is maintained. 
