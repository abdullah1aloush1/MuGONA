# Multiple-Goals-Ordering-Navigation-Algorithm
Multiple-Goals-Ordering-Navigation-Algorithm
MuGONA is a combined algorithm that takes an image map file as an input. The map should contain only black and white coloured areas where white coloured areas represent navigatable passages for the robot while black areas represent blocked passages that the robot can not navigate through.
The algorithm works on finding a near-optimal visiting configuration (order) to visit the provided goal nodes. Then, it connects the ordered goal nodes by navigatable path sections providing sampled path coordinates along the whole path.
The algorithm proceeds to process the generated overall path and smooth it using Cubic-splines to be more appropriate for real-life robot navigation.
The algorithm finally outputs a list of the coordinates of the smoothed (or original) overall path and plots it as a figure.
