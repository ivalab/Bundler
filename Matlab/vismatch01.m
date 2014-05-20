%=============================== vismatch01 ==============================
%
%  script vismatch01.m
%
%
%  Visualize the keypoints found using SIFT.  Plots the image and then
%  plots a vector indicating the direction and scale of the attribute.
%
%=============================== vismatch01 ==============================

There will be two variables indicating the two frames.
Check using exist command for the existence of a matches_XXX_YYY.mat
file.  If it does not exist kick out an error using the error command.
Otherwise, move on ...

As with part01, use the impathreader to load a file.
Use variable indicating which frame to load to do so.
Use the second variable saying which would be the second image.
Load that frame too.  Use the jumpto member function.

Then load the matching information.

matchdat = load( properfilenamevariable );

which will then dump the contents as a structure into siftdat
variable.  Access to the data is like with vispts01.

All you should use is the pairwise image locations, not the keypoint
indices.  If you'd like you can print out text by the point in
the first image indicating the keypoint index.  But, I am getting 
ahead of myself.


Take the two images and concatenate them so that they are one
side-by-side image.  To get the coordinate to match, you will have
to add to the second coordinate pair the horizontal offset associated
with the first image.  Add in this offset to the second set of key point
coordinates, then loop through the matches and plot lines connecting them
like is done in lots of publications on the matching.  I think you know
what's up.

That's it.  You can probably recycle some code from somewhere, but you'd
have to adapt it to the matchdat structure.

%
%=============================== vismatch01 ==============================
