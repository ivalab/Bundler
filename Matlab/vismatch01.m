%=============================== vismatch01 ==============================
%
%  script vismatch01.m
%
%
%  Visualize the keypoints found using SIFT.  Plots the image and then
%  plots a vector indicating the direction and scale of the attribute.
%
%=============================== vismatch01 ==============================


NumFrame1 = 5;
NumFrame2 = 9;

IMAGEPATH = 'H:/ioannis6/Documents/projects/Bundler/examples/ET'; %=========Changed, you may rechange it
IMAGETYPE = 'jpg';
  

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
 
colormap = {'b','r','m','y','g','c'};

figure,
 hold on
  set(gca,'ydir','reverse')
  set(gcf,'position',[567    66   854   709])

  NGap = 10;
  gap = ones(size(I1,1),NGap,3)*0.9/2;

  image([I1,gap,I2]);
  axis image off;


  siftdatIm2.keyp(1,:) = siftdatIm2.keyp(1,:)+size(I1,2)+NGap;

  hold on
   for i=1:size(matchdat.matches,2)

     plot([siftdatIm1.keyp(1,matchdat.matches(1,i)),siftdatIm2.keyp(1,matchdat.matches(2,i))],[siftdatIm1.keyp(2,matchdat.matches(1, ...
						  i)),siftdatIm2.keyp(2,matchdat.matches(2,i))],colormap{mod(i,6)+1});

   end

  hold off


