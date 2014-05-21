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

function vispts01(IMAGEPATH, IMAGETYPE, NumFrame)

  %Load Image
  ih = impathreader(IMAGEPATH, ['*.' IMAGETYPE], []);
  
  if exist([ IMAGEPATH '/keypts' num2str(NumFrame,'%04d') '.mat'])
    siftdat = load([ IMAGEPATH '/keypts' num2str(NumFrame,'%04d') '.mat']);
  else
    error('This file does not exist');
  end
  
  I = ih.jumpto(NumFrame);
   
  
  
   

%   The first two elements are the location in the image.
%   The third is the orientation.  Turn it into a 2x1 vector (unit length).
%   The fourth is the scale.  Use it to scale the vector.
    

  % Plot the Image
    imshow(I);
    hold on
    quiver(siftdat.keyp(1,:),siftdat.keyp(2,:),siftdat.keyp(3,:)./norm(siftdat.keyp(3,:)).*siftdat.keyp(4,:),siftdat.keyp(4,:),1);
    hold off
    


%
%================================ vispts01 ===============================
