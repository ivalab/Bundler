
Alright, so this is a record of how to resolve the current issues with
regards to the latest milestone.  The goal is to get something like Bundler
working in Matlab, and then later into it's own pipeline for use with
3D reconstruction of civil infrastructure via SFM or for indoor localization
via RGDB SLAM.


1. Perform the feature detection, feature description, and matching.
  This looks to be possible using mexopencv.  The big questions is
  how to store it since this can get massive.  In memory, or to file?
  I think that each feature set detected can be saved to a Matlab file
  and then loaded individually when looking to match pairwise across
  images.  
  
  Can it be done only if the data gets too big?  Maybe check image
  file sizes and quantity of images, then decide based on some kind
  of estimate.

2. Need to read the exif information.  Bundler used jhead, which can
  be found for just about any linux distribution or compiled from
  scratch (http://www.sentex.net/~mwandel/jhead/index.html).
  In Matlab, we can utilize the imfinfo to get the exif data from
  the jpeg files.

3. Once all the matches are gotten and stored, the next step is to
  use bundler to perform the bundle adjustment.  For that bundler
  needs to be unpacked and mexed up.  the proper output needs to
  be figured out.  It looks to be:

  a. the camera paramters for each camera.
  b. the 3D points for the views that have sufficient matches.
  c. which cameras view which points.

  It looks like all of the points should somehow be labelled.
  I think this can be done when performing the matching in the
  first part.  Need to provide a match structure.

  The question here is: why not use openCV bundle adjustment instead
  of Bundler.  That might actually be easier.  Or any other set of
  code.  For now, we have a pipeline that works and needs to be
  recreated.  Once we get it working, we can explore how to repackage
  it with alternative algorithms.


That seems to be it.  Not too complicated.  Of course, since we are
interested in reconstruction civil infrastructure, the next step
will be to perform a second, informed, higher fidelity pass with
finer features.  Probably a good idea would be to rectify the pairwise
images.  Here, we want to create special keyframes that correspond to
unique views.  Identifying these might actually be an interesting
optimization problem in and of itself.  The idea would be to define the
minimal set of central viewpoints from a collection of viewpoints.
We can define centrality to be based on some mathematical formulation or
review the literature to see if it is already defined.


Other BA and CV methods/code:
http://grail.cs.washington.edu/projects/mcba/ (PBA)
http://users.ics.forth.gr/~lourakis/sba/ (Bundler's version)
http://www.uco.es/investiga/grupos/ava/node/39 (cvsba - SBA for OpenCV)
http://www.doc.ic.ac.uk/~ajd/software.html (SceneLib - Davison, interesting
links).
