function rot_coords = RotateArea(center,width,height,angle)
    %theta = angle*(pi/180);
    theta = -angle;
    coords = [center(1)-(width/2) center(1)-(width/2) center(1)+(width/2)  center(1)+(width/2);...
          center(2)-(height/2) center(2)+(height/2) center(2)+(height/2)  center(2)-(height/2)];
    R = [cos(theta) sin(theta);...
        -sin(theta) cos(theta)];
    rot_coords = R*(coords-repmat(center,[1 4]))+repmat(center,[1 4]);
    rot_coords(:,5)=rot_coords(:,1);
    c = [0 0.4470 0.7410];
    line(rot_coords(1,:),rot_coords(2,:),'color',c,'LineWidth',1);

end