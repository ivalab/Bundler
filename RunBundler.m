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

%==[1] Setup the environment.
BASE_PATH = mfilename('fullpath');
if (ispc)
  BASE_PATH = BASE_PATH(1:find(BASE_PATH == '\',1,'last'));
  BUNDLER = 'Bundler';
elseif (isunix)
  BASE_PATH = BASE_PATH(1:find(BASE_PATH == '/',1,'last'));
  BUNDLER = 'bundler';
end

KEYMATCH = [BASE_PATH '/KeyMatchFull'];
BUNDLER  = [BASE_PATH '/' BUNDLER];

disp('Need to write the ToSift script');
%TO_SIFT=$BASE_PATH/bin/ToSift.sh
disp('Need to write the extract_focal script or ignore it.');
% EXTRACT_FOCAL=$BASE_PATH/bin/extract_focal.pl
%TODO: Matlab has exif info reading in it.  Should be able to
%TODO:  rewrite perl script as Matlab "easily."
%TODO: Otherwise, just ignore for now.  All focal length does
%TODO:  is alter scale.

%==[2] Perform some pre-processing.


IMAGE_DIR = './';

%== Convert to jpeg -- Is this really necessary for Matlab?
%{
# Rename ".JPG" to ".jpg"
for d in `ls -1 $IMAGE_DIR | egrep ".JPG$"`
do 
  mv $IMAGE_DIR/$d $IMAGE_DIR/`echo $d | sed 's/\.JPG/\.jpg/'`
done
%}

%== Write out list of images for passing to sift executable.
%   Is this really necessary if we can invoke from Matlab?
%{
# Create the list of images
find $IMAGE_DIR -maxdepth 1 | egrep ".jpg$" | sort > list_tmp.txt
$EXTRACT_FOCAL list_tmp.txt
cp prepare/list.txt .
%}

%== Execute SIFT feature point search.
%{
# Run the ToSift script to generate a list of SIFT commands
echo "[- Extracting keypoints -]"
rm -f sift.txt
$TO_SIFT $IMAGE_DIR > sift.txt 

# Execute the SIFT commands
sh sift.txt
%}

%== Perform matching of SIFT features across images.
%{
# Match images (can take a while)
echo "[- Matching keypoints (this can take a while) -]"
sed 's/\.jpg$/\.key/' list_tmp.txt > list_keys.txt
sleep 1
echo $MATCHKEYS list_keys.txt matches.init.txt
$MATCHKEYS list_keys.txt matches.init.txt
%}

%== Execute bundler on the images.
%{
# Generate the options file for running bundler 
mkdir bundle
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
