%mat = imread('datasets/sequence_01/images/00000.jpg', 'JPEG');
%imshow(mat);

dataset_prefix = 'datasets/sequence_01';
num_images = 5000;


for index = (0:(num_images-1))
    image_filename = sprintf('%s/images/%05d.jpg', dataset_prefix, index);
    mat = imread(image_filename, 'JPEG');
    imshow(mat);
    [Gx, Gy] = imgradientxy(mat);
    
    %imshow(Gy);
end