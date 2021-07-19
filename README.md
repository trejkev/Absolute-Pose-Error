# Absolute-Pose-Error
Absolute Pose Error calculator from ROS bags topic messages. When running a new bag analysis, all the initial variables must be set to true, so that all the parts of the algorithm run. The bag path has to be settled into bagFilePath variable, at the beginning of the script.

The script will generate three images, examples of them are included into the repo.

APE formula used for each individual reading is: sqrt((x_error)^2+(y_error)^2)
