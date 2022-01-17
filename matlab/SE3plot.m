function SE3plot(pose,d,width)
% function SE3plot(pose,d,l) 
% this function draw body axis to defined position p with each length d and
% thickness l 
% If pose = SE3 is cell, we will draw the whole sequence 

hold on
if iscell(pose)
    for n = 1:length(pose)
        
        p = pose{n}.t; R = pose{n}.R;
        plot3([p(1) p(1)+d*R(1,1)],[p(2) p(2)+d*R(2,1)],[p(3) p(3)+d*R(3,1)],'r','LineWidth',width)
        plot3([p(1) p(1)+d*R(1,2)],[p(2) p(2)+d*R(2,2)],[p(3) p(3)+d*R(3,2)],'g','LineWidth',width)
        plot3([p(1) p(1)+d*R(1,3)],[p(2) p(2)+d*R(2,3)],[p(3) p(3)+d*R(3,3)],'b','LineWidth',width)
    end
        
else
    p = pose.t; R = pose.R;
    plot3([p(1) p(1)+d*R(1,1)],[p(2) p(2)+d*R(2,1)],[p(3) p(3)+d*R(3,1)],'r','LineWidth',width)
    plot3([p(1) p(1)+d*R(1,2)],[p(2) p(2)+d*R(2,2)],[p(3) p(3)+d*R(3,2)],'g','LineWidth',width)
    plot3([p(1) p(1)+d*R(1,3)],[p(2) p(2)+d*R(2,3)],[p(3) p(3)+d*R(3,3)],'b','LineWidth',width)
    
end
end