function drawGaussian2D(Sigma, mu)

mu = mu';

 

x1 = -10:.2:10; x2 = -10:.2:10;
[X1,X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(x2),length(x1));
surf(x1,x2,F);
caxis([min(F(:))-.5*range(F(:)),max(F(:))]);
axis([-10 10 -10 10 0 1])
xlabel('x1'); ylabel('x2'); zlabel('Probability Density');

axis([-8 8 0 8 0 1])
 view(0,90)
  pbaspect([2 1 1])
 

end

