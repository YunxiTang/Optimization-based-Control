function link = link_build(config)
%link: to establish a link of a serial robot
%   cong = [joint_type,a,alpha,d,theta]
%   joint_type: 'r' --> revolute joint; 'p' --> prismatic joint
%   a --> the length of link
%   alpha --> the twist angle of link
%   d --> the offset of link
%   theta --> the revolute angle 
flag = 1;

if config(1) == 'r' || config(1) == 'p'
else
    disp('Wrong joint type!');
    flag = 0;
end

if config(2) >= 0
else
    disp('The length of link should be non-negative.');
    flag = 0;
end
    
if  config(4)>= 0
else
    disp('The offset of link should be non-negative.');
    flag = 0;
end
if flag
    link.type = config(1);
    link.a =  config(2);
    link.d = config(4);
    link.alpha = config(3);
    link.theta =  config(5);
    
    %create a transformation matrix for this link
    link.T = Rot('X',link.alpha)*Trans('X',link.a)*Rot('Z',link.theta)*Trans('Z',link.d);
    
else
    disp('Failed to build a link');

end

