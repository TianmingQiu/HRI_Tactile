close all; clear; clc;

recordTrajectory;



fid = fopen('x.txt','wt');
fprintf(fid, '%g\n', x);
fclose(fid);
fid = fopen('y.txt','wt');
fprintf(fid, '%g\n', y);
fclose(fid);
