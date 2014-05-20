%=============================== RunBundler ==============================
%
%  script RunBundler.m
%
%
%  A Matlab transcoding of the RunBundler bash script for Windows systems
%  that do not have cygwin installed and owners who don't want to bother
%  given that Matlab is a scripting language.  Heck, I would imagine that
%  it would work on whatever system has bundler executables compiled.  It is
%  a modification of Noah Snavely's original bash script right now, and thus
%  this version inherits whatever copyrights were of the original version.
%
%  The script basically does the feature detection and description work,
%  preps the image pairs and thei matches, then sends all of the files
%  to bundler for processing.  Make sure to set the proper path variables.
%
%=============================== RunBundler ==============================

%
%  Name:        RunBundler.m
%
%  Author:      Patricio A. Vela,           pvela@gatech.edu
%               Ioannis Anagnostopoulous
% 
%  Created:     2014/05/15
%  Modified:    2014/05/14
%
%
%  TODO:        For now it is a script.  Might want to make a function.
%
%  DEPENDENCIES:
%       VLFeat          http://www.vlfeat.org/
%       
%=============================== RunBundler ==============================

%{
# Usage: RunBundler.sh [image_dir]
#
# The image_dir argument is the directory containing the input images.
# If image_dir is omitted, the current directory is used.
#

# Set this variable to your base install path (e.g., /home/foo/bundler)
# BASE_PATH="TODO"
%}

%==[0] Options.
visOutput = false;



%==[1] Setup the environment.
BASE_PATH = mfilename('fullpath');
if (ispc)
  BASEPATH = BASEPATH(1:find(BASEPATH == '\',1,'last'));
  BUNDLER = 'Bundler';
elseif (isunix)
  BASEPATH = BASEPATH(1:find(BASEPATH == '/',1,'last'));
  BUNDLER = 'bundler';
end

KEYMATCH = [BASEPATH '/KeyMatchFull'];
BUNDLER  = [BASEPATH '/' BUNDLER];

%disp('Need to write the ToSift script');
%TO_SIFT=$BASEPATH/bin/ToSift.sh
%disp('Need to write the extract_focal script or ignore it.');
% EXTRACT_FOCAL=$BASEPATH/bin/extract_focal.pl
%TODO: Matlab has exif info reading in it.  Should be able to
%TODO:  rewrite perl script as Matlab "easily."
%TODO: Otherwise, just ignore for now.  All focal length does
%TODO:  is alter scale.

%==[2] Perform SIFT detection and description on all loadable images.
%       Save them to a Matlab file for loading as needed.
IMAGEPATH = './';
IMAGETYPE = 'jpg';

parms.improcessor = improcessor_basic('rgb2gray',{});
ih = impathreader(IMAGEPATH, ['*.' IMAGETYPE], [], parms);

while (ih.isNext())
  I = ih.next();
  [keyp, desc] = vl_sift(I);

  outfile = [ IMAGEPATH '/keypts' num2str(ih.frame(),'%04d') '.mat'];
  save(outfile, 'keyp', 'desc');
end

%==[3] Perform matching of SIFT features across images.
ih.reset();

%TODO: What do to here?  Start with dumb approach, all pairwise.
%{
For each image in sequence, try to match against other images in sequence
(not yet matched against).  Consider the two images to be "connected" if
more than some percentage of feature points match (read Snavely thesis
to see if other tricks were used, or read the Bundler journal paper; should
be quick since you just want implementation details).  

Keep track of connectivity and output to Matlab structure the pairwise
matches for that given frame.  Should there be a separate file for each
pairwise match, or one file for all given a frame as per Bundler?

For now, save as matches_XXXX_YYY using the same kind of outfile technique
as above.  So, one file for each pair of images that match up.

Save the matching keypoint indices from the two frames in a variable
called inds (2xN variable, with first column being index to first image
keypoint list and second column being index to second image keypoint list).  
Save the matching keypoint image locations in a variable called pts (2x2xN)
variable.  You might or might not need the cat command.
%}

%==[4] Execute bundler on the images.
%{
# Generate the options file for running bundler 

%TODO: Check for bundle sub-dir.  If not exist, make.
mkdir bundle
%TODO: Create bundle/options.txt (open for writing with clear)
rm -f options.txt

echo "--match_table matches.init.txt" >> options.txt
echo "--output bundle.out" >> options.txt
echo "--output_all bundle_" >> options.txt
echo "--output_dir bundle" >> options.txt
echo "--variable_focal_length" >> options.txt
echo "--use_focal_estimate" >> options.txt
echo "--constrain_focal" >> options.txt
echo "--constrain_focal_weight 0.0001" >> options.txt
echo "--estimate_distortion" >> options.txt
echo "--run_bundle" >> options.txt
%}

%{
# Run Bundler!
echo "[- Running Bundler -]"
rm -f constraints.txt
rm -f pairwise_scores.txt
$BUNDLER list.txt --options_file options.txt > bundle/out
%}

disp('[- Done -]');

%==[5] Load main output file and visualize, if specified.
if (visOutput)

end
