%================================ vispts01 ===============================
%
%  script vispts01.m
%  
%  input: IMAGEPATH: Path of the images
%         IMAGETYPE: Type of image e.g. 'jpg'
%         frame: The image to be visualized
%  
%  Visualize the keypoints found using SIFT.  Plots the image and then
%  plots a vector indicating the direction and scale of the attribute.
%
%================================ vispts01 ===============================

NumFrame = 5;


IMAGEPATH = 'H:/ioannis6/Documents/projects/Bundler/examples/ET'; %=========You may Change it
IMAGETYPE = 'jpg';

%Load Image
ih = impathreader(IMAGEPATH, ['*.' IMAGETYPE], []);
  
if exist([ IMAGEPATH '/keypts' num2str(NumFrame,'%04d') '.mat'])
  siftdat = load([ IMAGEPATH '/keypts' num2str(NumFrame,'%04d') '.mat']);
else
  error('This file does not exist');
end
  
I = ih.jumpto(NumFrame);

vector = [cos(siftdat.keyp(3,:)); sin(siftdat.keyp(3,:))];

% Plot the Image
imshow(I),
 hold on
  quiver(siftdat.keyp(1,:),siftdat.keyp(2,:),vector(1,:).*siftdat.keyp(4,:),vector(2,:).*siftdat.keyp(4,:),1);
 hold off
    


%
%================================ vispts01 ===============================
