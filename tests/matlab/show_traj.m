tj =load('~/.ros/result.txt'); 
tj = alignZ(tj(:,[2,3,4]));
plot(tj(:,1), tj(:, 2))

function X = alignZ(XYZ)
xyz0=mean(XYZ);
A=bsxfun(@minus,XYZ,xyz0); %center the data
[~,~,V]=svd(A,0);
XYZ = A*V;
angle = XYZ(floor(size(XYZ,1)/4),1:2) - XYZ(1,1:2);
angle = atan2(angle(2), angle(1));
R = [cos(angle), sin(angle), 0; -sin(angle), cos(angle), 0; 0, 0, 1];
XYZ = XYZ * R';
X = XYZ - XYZ(1,:);
end