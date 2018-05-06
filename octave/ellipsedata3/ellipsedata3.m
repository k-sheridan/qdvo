function [elv,elf] = ellipsedata3(covmat,center,numpoints,sigmarule,varargin)

%% Ellipsedata3 V1.000
%
% Constructs data points of ellipses representing contour curves of 3-dimensional
% Gaussian distributions with any covariance and mean value.
%
%% Example
%
% In this example, the funcion ellipsedata constructs one ellipsoid (3-dimensional
% ellipse) of 400 points representing the contour curve corresponding to a standard
% deviation of 1.5 for a 3-dimensional Gaussian distribution with covariance matrix given by
% [4,1,1;1,2,1;1,1,1] and mean value given by [3,3,3].
%
% [elv,elf]=ellipsedata3([4,1,1;1,2,1;1,1,1],[3,3,3],20,1.5);
%
% The positions of the vertices are stored in elv, and can be plotted as follows
%
% scatter3(elv(:,1),elv(:,2),elv(:,3));
%
% The faces (vertices forming a patch) are stored in elf, and can be plotted as follows
%
% patch('Faces',elf,'Vertices',elv,'FaceColor','white','EdgeColor','black');
%
%% Input arguments
%
%   covmat:
%       Covariance matrix of a 3-dimensional Gaussian distribution. Must be of
%       size 3x3, symmetric and positive definite. If the format is not
%       correct, an error is triggered. If the matrix is not symmetric, it
%       is symmetrized by adding its transpose and dividing by 2.
%
%   center:
%       The center (mean value) of the 3-dimensional Gaussian distribution. If
%       the format is not correct, it is set to [0,0,0].
%
%   numpoints:
%       Square root of the number of points that each ellipsoid will be composed of.
%       Must be a positive integer number. If it is not numeric or positive, it is
%       set to 20. If it is not integer, it is converted to integer using
%       the function ceil.
%
%   sigmarule:
%       The proportion of standard deviation surrounded by each ellipsoid.
%
%   varargin (later assigned to "zeroprecision"):
%       A real number indicating the maximum difference after which two
%       numbers are considered different. This value is used for assessing
%       whether covmat is symmetric. If not specified, it is set to 1E-12
%
%% Output arguments
%
%   elv:
%       Matrix in which each row represents the positions of the points (vertices)
%       that constitute the ellipsoid.
%
%   elf:
%       Matrix with the indices of the vertices that constitute each face (patch)
%       of the ellipsoid.
%
%% Version control
%
% V1.000: First version. Some points in elv are repeated. For simplicity,
% they are left as such.
%
%
%% Please, report any bugs to Hugo.Eyherabide@cs.helsinki.fi
%
% Copyright (c) 2014, Hugo Gabriel Eyherabide, Department of Mathematics
% and Statistics, Department of Computer Science and Helsinki Institute
% for Information Technology, University of Helsinki, Finland.
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
%
% 1. Redistributions of source code must retain the above copyright
% notice, this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright
% notice, this list of conditions and the following disclaimer in the
% documentation and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
% HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
% SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
% TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
% OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
% OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


%% Check format of initial parameters
warningmessage=@(varname)warning(['Format of "' varname '" incorrect. Setting "' varname '" to default.']);

if isempty(varargin) || ~isnumeric(varargin{1}) || length(varargin{1})~=1,
    zeroprecision=1E-12;
else
    zeroprecision=varargin{1};
end

if zeroprecision<0, zeroprecision=-zeroprecision; end

if ~isnumeric(covmat) || size(covmat,1)~=size(covmat,2) || det(covmat)<0,
    error('The argument "covmat" is not a covariance matrix');
end

if mean(mean(abs(covmat-covmat')))>zeroprecision,
    warning('The matrix "covmat" is not symmetric, and it has been symmetrized by adding its transpose and divided by 2');
    covmat=(covmat+covmat')/2;
end

if ~isnumeric(center) || length(center)~=3,
    warningmessage('center');
    center=[0;0;0];
end

if ~isnumeric(numpoints) || length(numpoints)~=1 || numpoints<1,
    warningmessage('numpoints');
    numpoints=20;
end

if ~isnumeric(sigmarule),
    warningmessage('sigmarule');
    sigmarule=3;
end


%% Calculations start here

% Converting input arguments into column vectors
center=center(:)';
sigmarule=sigmarule(:)';
numpoints=ceil(numpoints);

% Calculates principal directions(PD) and variances (PV)
[PD,PV]=eig(covmat);
PV=diag(PV).^.5;

% Chooses points
theta=repmat(linspace(0,pi,numpoints)',1,numpoints);
phi=repmat(linspace(0,2*pi,numpoints),numpoints,1);

% Construct ellipse
theta=theta(:);
phi=phi(:);
elv=[cos(phi).*sin(theta),sin(phi).*sin(theta),cos(theta)];
elv=elv*diag(PV)*PD'*sigmarule;
elv=elv+repmat(center,numpoints^2,1);

npm1=numpoints-1;
elf=zeros(npm1^2,4);
indtpf=@(ip,it)(ip-1)*npm1+it;
indtpv=@(ip,it)(ip-1)*numpoints+it;
for ip=1:npm1,
    for it=1:npm1,
        elf(indtpf(ip,it),:)=[indtpv(ip,it),indtpv(ip+1,it),indtpv(ip+1,it+1),indtpv(ip,it+1)];
    end
end

end

