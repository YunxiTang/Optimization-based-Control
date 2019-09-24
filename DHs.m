function robot_T = DHs(dh_matrix)
%   create DH Table based on a provided matrix
%   if a manipulator has m links, the dh_matrix should be m x 5
[row,col] = size(dh_matrix);
if col == 5
    for i = 1:row
        type = dh_matrix(i,1);
        a = dh_matrix(i,2);
        alpha = dh_matrix(i,3);
        d = dh_matrix(i,4);
        theta = dh_matrix(i,5);
    
        Link = link_build(type,a,alpha,d,theta);
        robot_T = Link.T;
    end
else 
    disp('The column should be 5!');
end
end



    
