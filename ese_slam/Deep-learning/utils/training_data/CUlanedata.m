imds = datastore(fullfile('/media/jh/HDD/Data/CULanedata/laneseg_label_w16'),...
    'IncludeSubfolders', true,'FileExtensions', '.png','Type', 'Image');
root = '/media/jh/HDD/Data/CULanedata';

%%
imagfile = imds.Files;
%%
file_ ={};
SE = strel('disk',4);
for j = 1 : length(bin)
    filename = ['/bin_image/bin_image_' num2str(j,'%05d') '.png'];
    imag  = imread([root bin{j}]);
    label = (imag>0)*255;
    label = imdilate(label,SE);
    imwrite(label,[root filename]);
    file_{j,1}=filename;
    j
end
%%
f = fopen('bin_image_list.txt','w');
f.write(ans,'delimeter','\n');
%%
file_thin ={};
SE = strel('disk',5);
for j = 1 : length(bin)
    filename = ['/bin_thin2/bin_thin_' num2str(j,'%05d') '.png'];
%     imag  = imread([root bin{j}]);
%     label = (imag>0)*255;
%     label = imerode(label,SE);
%     imwrite(label,[root filename]);
    file_thin{j,1}=filename;
    j
end
