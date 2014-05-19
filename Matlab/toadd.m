
%Psuedocode for slightly more nuanced approach.
%{
PHASE 1:

Take first image and try to match against next in sequence until matches
fall below a certain specified percentage.  The frames matched to now
form what's called a "track." 

Perform matching within the track.

Take the image that terminated the track and repeat the procedure above
until there are no images left.
%}

%{
PHASE 2:

Go through tracks and see if can pick up to 2-3 "central" images.

Perform pairwise matching of all central images.  Establish some
kind of connectivity measure.  Order all tracks according to 
connectivity.

%}

