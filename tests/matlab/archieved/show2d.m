D = load('/home/jiawei/.ros/seq06.txt');
D_p = D(:,2:4);
[coeff,score] = pca(D_p);
d2d = score(:,1:2);
plot(d2d)
plot(d2d(:,1), d2d(:,2))