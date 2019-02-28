dir = '/home/jiawei/Dropbox/IROS/eval/kitti';
results_dir = strcat(dir, '/results/poses');

out_dir = strcat(dir, '/output');
system(['rm -rf ', out_dir]);
mkdir(out_dir)
out_results_dir = strcat(out_dir, '/results');
mkdir(out_results_dir)
out_formatSpec = '%f %f %f %f %f %f %f %f %f %f %f %f\n';

for i=00:10
    result_file = strcat(results_dir, sprintf('/%02d.txt',i));
    result = tum2kitti(load(result_file));

    out_results_file = fopen(strcat(out_results_dir, sprintf('/%02d.txt',i)), 'w');
    fprintf(out_results_file, out_formatSpec, result');
    fclose(out_results_file);
end


function D_K = tum2kitti(D)
D_c = D(:,2:4);
D_q = D(:,[8,5:7]);
D_R = quat2rotm(D_q);
D_K = zeros(length(D), 12);
for i=1:length(D)
    D_K(i, 1:3)  = D_R(1,:,i);
    D_K(i, 4)    = D_c(i,1);
    D_K(i, 5:7)  = D_R(2,:,i);
    D_K(i, 8)    = D_c(i,2);
    D_K(i, 9:11) = D_R(3,:,i);
    D_K(i, 12)    = D_c(i,3);
end
end