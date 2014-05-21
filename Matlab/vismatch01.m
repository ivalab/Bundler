%=============================== vismatch01 ==============================
%
%  script vismatch01.m
%
%
%  Visualize the keypoints found using SIFT.  Plots the image and then
%  plots a vector indicating the direction and scale of the attribute.
%
%=============================== vismatch01 ==============================


%=== Keep whatever method for visualization you want
function vismatch01(IMAGEPATH,NumFrame1,NumFrame2)
  

  if (NumFrame1 > NumFrame2)
    temp = NumFrame1;
    NumFrame1 = NumFrame2;
    NumFrame2 = temp;
  end
  
  if exist([IMAGEPATH '/matches' '_' num2str(NumFrame1,'%04d') '_' num2str( NumFrame2,'%04d') '.mat'])
    matchdat = load([IMAGEPATH '/matches' '_' num2str(NumFrame1,'%04d') '_' num2str( NumFrame2,'%04d') '.mat']);
    siftdatIm1 = load([IMAGEPATH '/keypts' num2str(NumFrame1,'%04d') '.mat']);
    siftdatIm2 = load([IMAGEPATH '/keypts' num2str(NumFrame2,'%04d') '.mat']);
    
  else
    error('The file does not exist');  
  end
  
  ih = impathreader(IMAGEPATH, ['*.' IMAGETYPE], []);
  I1 = ih.jumpto(NumFrame1);
  I2 = ih.jumpto(NumFrame2);
  
  pts = ([siftdatIm1.keyp(1:2,matchdat.matches(1,:));siftdatIm2.keyp(1:2,matchdat.matches(2,:))]');

  % Plot matches
 
  figure,
   colormap = {'b','r','m','y','g','c'};
   height = max(size(I1,1),size(I2,1));
   I1_ratio = height/size(I1,1);
   I2_ratio = height/size(I2,1);
   I1 = imresize(I1,I1_ratio);
   I2 = imresize(I2,I2_ratio);
   points1 = pts(:,1:2) .* I1_ratio;
   points2 = pts(:,3:4) .* I2_ratio;
   points1 = [points1(:,2) points1(:,1)];
   points2 = [points2(:,2) points2(:,1)];
   match_img = zeros(height, size(I1,2)+size(I2,2), size(I2,3));
   match_img(1:size(I1,1),1:size(I1,2),:) = I1;
   match_img(1:size(I2,1),size(I1,2)+1:end,:) = I2;
   imshow(uint8(match_img));
   hold on;
     for i=1:size(points1,1)
        plot([points1(i,2) points2(i,2)+size(I1,2)],[points1(i,1) points2(i,1)],colormap{mod(i,6)+1});
     end

   hold off;

  

  figure,
   hold on
    set(gca,'ydir','reverse')
    set(gcf,'position',[567    66   854   709])

    NGap = 50;
    gap = ones(size(I1,1),NGap,3)*0.9/2;

    image([I1,gap,I2]);
    axis image off;


    siftdatIm2.keyp(1,:) = siftdatIm2.keyp(1,:)+size(I1,2)+NGap;

    hold on
     for i=1:size(points1,1)

       plot([siftdatIm1.keyp(1,matchdat.matches(1,i)),siftdatIm2.keyp(1,matchdat.matches(2,i))],[siftdatIm1.keyp(2,matchdat.matches(1, ...
						  i)),siftdatIm2.keyp(2,matchdat.matches(2,i))],colormap{mod(i,6)+1});

     end

    hold off

end

% All you should use is the pairwise image locations, not the keypoint
% indices.  If you'd like you can print out text by the point in
% the first image indicating the keypoint index.  But, I am getting 
% ahead of myself.
% 
% 
% Take the two images and concatenate them so that they are one
% side-by-side image.  To get the coordinate to match, you will have
% to add to the second coordinate pair the horizontal offset associated
% with the first image.  Add in this offset to the second set of key point
% coordinates, then loop through the matches and plot lines connecting them
% like is done in lots of publications on the matching.  I think you know
% what's up.
% 
% That's it.  You can probably recycle some code from somewhere, but you'd
% have to adapt it to the matchdat structure.

%
%=============================== vismatch01 ==============================
