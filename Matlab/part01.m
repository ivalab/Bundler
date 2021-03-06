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


% 
% Ioannis, go to the RunBundler.m file in the directory above this one
% and see steps [2] and [3].  Flesh them out based on the comments in
% the file, on the original RunBundler.sh bash script, and on the
% original paper by Snavely et al. (or his thesis which can be found
% in the Literature directory of the SLAM repository).
% 
% You should copy most of the code from parts [2] and [3] and before
% here,except for anything that looks to be extensively commented out.
% The minimal needed for you and I to run on our rspective machines.
% 
% By the way, you should copy Bundler to the SLAM directory, as in
% 
%   blahblah/blah/SLAM/Bundler
% 
% The .gitignore file will automatically ignore it and won't synchronize
% it with the SLAM repository.  
% 
% 
% Medium to Longer term:
% 
% If you get ssh-key and ssh-add working in your bash script or in Windows,
% then you should be able to push and pull to/from multiple repositories
% without any problems.  You will have to look it up online.  I know how to do
% it in linux but not in Windows.  It's pretty easy (in linux at least), so I
% figure it won't take too long.  Once you manage to get the ssh key daemon to
% run, then you won't have to enter a password but once.


%
%================================= part01 ================================

%==[0] Setup the environment.
%

%--[0.1] Directories where things will be found.
BASEPATH = mfilename('fullpath'); 
if (ispc)
  IMAGEPATH = 'H:/ioannis6/Documents/projects/bundler/examples/ET'; 
  MATLIBPATH = 'H:/ioannis6/Documents/Matlab';
  VLFEATPATH = 'H:/ioannis6/Documents/projects/vlfeat'; % Now you change!

  BASEPATH = BASEPATH(1:find(BASEPATH == '\',1,'last'));
  BUNDLER = 'Bundler';
elseif (isunix)
  IMAGEPATH = '~/projects/SLAM/Bundler/examples/ET';
  MATLIBPATH = '~/Matlab';
  VLFEATPATH = '~/src/cvLibs/vlfeat';

  BASEPATH = BASEPATH(1:find(BASEPATH == '/',1,'last'));
  BUNDLER = 'bundler';
end

%--[0.2] Include libraries.
MATLIBS = {'improcessors', 'readers'};
for ii=1:length(MATLIBS)
  addpath([MATLIBPATH '/' MATLIBS{ii}]);
end

%--[0.3] Include vlfeat
addpath([VLFEATPATH '/toolbox']);
run('vl_setup');        

%
%==[1] Options.
%
visOutput = false;
IMAGETYPE = 'jpg';

%
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

parms.improcessor = improcessor_basic('rgb2gray',{},'single',{});     
ih = impathreader(IMAGEPATH, ['*.' IMAGETYPE], [], parms);     
%IOANNIS: The impathreader did not incorporate improcessor like
%IOANNIS:  other readers do. My bad.  I corrected.  Now it runs
%IOANNIS:  the commands above (after loading the image and ) prior to 
%IOANNIS:  returning the final image.  It should be in the format
%IOANNIS:  needed by SIFT so you don't have to add annoying code
%IOANNIS:  for image conversion.  If you are clever you can use
%IOANNIS:  the improcessor to do lots of funky things.

while (ih.isNext())
  I = ih.next();
  [keyp, desc] = vl_sift(single(rgb2gray(I))); 

  outfile = [ IMAGEPATH '/keypts' num2str(ih.frame(),'%04d') '.mat']
  save(outfile, 'keyp', 'desc');
end


%==[3] Perform matching of SIFT features across images.

ih.reset();

%IOANNIS: Learn to use Matlab's inline functions to make life cleaner.
genFilename = @(ind)[IMAGEPATH '/keypts' num2str(ind,'%04d') '.mat'];
genMatchname = @(i1, i2)[IMAGEPATH '/matches_' num2str(i1,'%04d') '_' ...
                                               num2str(i2, '%04d') '.mat'];

%IOANNIS: You did not read about the impathreader.  It has more flexibility
%IOANNIS:  Than what you used.  Always read what you are using and
%IOANNIS:  understand what the interface and its abilities are.
iter = 1;
for i=1:ih.length();
  I1 = ih.jumpto(i);

  for j=i+1:ih.length();
    I2 = ih.jumpto(j);

    SiftdatIm1 = load(genFilename(i));
    SiftdatIm2 = load(genFilename(j));
  
    [matches, scores] = vl_ubcmatch(SiftdatIm1.desc,SiftdatIm2.desc);
     
    pts = ([SiftdatIm1.keyp(1:2,matches(1,:)); ...
                  SiftdatIm2.keyp(1:2,matches(2,:))]');
              
    outfile = genMatchname(i, j);
    save(outfile,'matches','scores','pts');
 
  end
end


%% Extract focal length


ImNames = dir(fullfile([IMAGEPATH '/' '*.jpg']));

num_output_images = 0;

for i=1:ih.length();
  %Extract image's info  
  exif = imfinfo([IMAGEPATH '/' ImNames(i).name]);
  
  % Grab Camera Model
  if (~isempty(exif.Make))
    Make = exif.Make;
    Model = exif.Model;
  end
  
  % Grab focal length
  if (~isempty(exif.DigitalCamera.FocalLength))
    FocalLength = exif.DigitalCamera.FocalLength;
  else
    FocalLength = 0;
  end
  
  % Grab dimensions
  x_res = exif.DigitalCamera.CPixelXDimension;
  y_res = exif.DigitalCamera.CPixelYDimension;
  
  %Grab CCD width from known cameras
  ccd = load('CCD_WIDTHS.MAT');
  Pos = strmatch([Make ' ' Model], ccd.ccd_models);
  
  if (~isempty(Pos))
    ccd_width = ccd.ccd_widths(Pos);
  %If the camera model's ccd width is not known then run jhead.exe
  else
    [status,cmdout] = system([BASEPATH 'bin/jhead.exe' [' ' IMAGEPATH '/' ImNames(i).name]]);
    Key   = 'CCD width    :';
    Index = strfind(cmdout, Key);
    if (~isempty(Index))
      ccd_width = sscanf(cmdout(Index(1) + length(Key):end), '%g', 1);
    %If jhead.exe cannot extract ccd width ask user for focal length
    else 
      prompt = 'Do you want to add the ccd width? [Y/N] ';
      str = input(prompt,'s');
      if isempty(str) || str =='N'
        str = 'N';
        ccd_width = 0;
     else
        prompt = 'Add the ccd width in mm ';
        ccd_width = input(prompt);
      end  
    end
  end
   
 % Check
  if (FocalLength == 0 || ccd_width == 0 || x_res == 0) 
    disp('Could not extract required fields');
  else
    if (x_res < y_res) 
	  tmp = x_res;
	  x_res = y_res;
	  y_res = tmp;
    end
    num_output_images = num_output_images + 1;
    
    %Extract focal length
    focal_pixels(num_output_images) = x_res * FocalLength/ccd_width;
          
 end
          
   
end

%TODO: Consider the two images to be "connected" if
% more than some percentage of feature points match (read Snavely thesis
% to see if other tricks were used, or read the Bundler journal paper; should
% be quick since you just want implementation details).  
% 
% SIFT
% Approximate nearest neighbors
% (https://github.com/jefferislab/MatlabSupport/tree/master/ann_wrapper)
% Fundamental Matrix for the pair using RANSAC, remove outliers etc. 
% If the number of remaining
% matches is less than twenty, all matches are removed.

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

