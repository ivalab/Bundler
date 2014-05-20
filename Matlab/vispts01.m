%================================ vispts01 ===============================
%
%  script vispts01.m
%
%
%  Visualize the keypoints found using SIFT.  Plots the image and then
%  plots a vector indicating the direction and scale of the attribute.
%
%================================ vispts01 ===============================


As with part01, use the impathreader to load a file.
The file should have a variable indicating which frame to load.
After loading the frame, load the data for that frame as written
by part01.m

The load invocation should be like:

siftdat = load( properfilenamevariable );

which will then dump the contents as a structure into siftdat
variable.  Access to the data is via: siftdat.keyp and siftdat.desc

All you should use is siftdat.keyp.

The first two elements are the location in the image.
The third is the orientation.  Turn it into a 2x1 vector (unit length).
The fourth is the scale.  Use it to scale the vector.

Plot the image.
Use Matlab's quiver command with scale = 1 to plot the SIFT key point
info on the image.  Make sure to hold on.


%
%================================ vispts01 ===============================
