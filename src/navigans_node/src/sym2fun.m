% INPUT : A symbolic matrix A and a string str
% OUTPUT : A file str.m that contains the function Afun which takes the
% symbolic variables of A as input and outputs the evaluation of A
% USAGE : sym2fun(A,'Afun');

% from https://www.mathworks.com/matlabcentral/newsreader/view_thread/79056
 
function sym2fun(A,str)
 
% Create a file str.m
fid = fopen([str '.m'],'w');
 
% Comment : date
date = fix(clock);
fprintf(fid,'%% %d-%d-%d %d:%d:%d\n\n',date);
 
% Create the function header
fprintf(fid,'function out = %s(%s)\n\n',str,findsym(A));
 
% Create the output
fprintf(fid,'out = [');
sA = size(A);
for row=1:sA(1)
    for col = 1:sA(2)
        fprintf(fid,'%s ',char(A(row,col)));
    end
    fprintf(fid,';...\n');
end
fprintf(fid,'];');
 
fclose(fid);