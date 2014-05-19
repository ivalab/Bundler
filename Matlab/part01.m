%================================= part01 ================================
%
%  script part01.m
%
%
%  A script to execute the first part of the Bundler process, and
%  basically that of any SfM pipeline, the pairwise image matching.
%  Here, the code effectively performs the matching in the RunBundler
%  script that corresponds to the SIFT detection, description extratcion
%  and the key point matching.
%
%================================= part01 ================================



Ioannis, go to the RunBundler.m file in the directory above this one
and see steps [2] and [3].  Flesh them out based on the comments in
the file, on the original RunBundler.sh bash script, and on the
original paper by Snavely et al. (or his thesis which can be found
in the Literature directory of the SLAM repository).

By the way, you should copy Bundler to the SLAM directory, as in

  blahblah/blah/SLAM/Bundler

The .gitignore file will automatically ignore it and won't synchronize
it with the SLAM repository.  


Medium to Longer term:

If you get ssh-key and ssh-add working in your bash script or in Windows,
then you should be able to push and pull to/from multiple repositories
without any problems.  You will have to look it up online.  I know how to do
it in linux but not in Windows.  It's pretty easy (in linux at least), so I
figure it won't take too long.  Once you manage to get the ssh key daemon to
run, then you won't have to enter a password but once.


%
%================================= part01 ================================
