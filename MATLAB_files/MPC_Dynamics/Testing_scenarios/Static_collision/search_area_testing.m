clear; clc;
%% Define The Global Map
mapmat = load('Maps/map1.mat');
mapmat = mapmat.mapmat;
%mapmat = mapmat.imageOccupancy;
map = occupancyMap(mapmat,5);
search_origin = [100; 100];
search_x = 20;
search_y = 5;

angle = 10*(pi/180);
x_rot_search = round([search_x*cos(angle) ; search_y*sin(angle)]);
y_rot_search = round([-search_x*sin(angle) ; search_y*cos(angle)]);


rotated_coor = RotateArea(search_origin,2*search_x,2*search_y,angle);
rotated_corners = rotated_coor(:,1:4);
rotated_corners_glob = rotated_corners(:,1:4)*0.2;

figure(1)
c = [0 0.4470 0.7410];
line(rotated_coor(1,:)*0.2,rotated_coor(2,:)*0.2,'color',c,'LineWidth',1);hold on

x_dir_o = [cos(angle) ;sin(angle)];
y_dir_o = [-sin(angle);cos(angle)];

if min(x_dir_o) ~= 0
x_dir = (round(x_dir_o/min(abs(x_dir_o))));
y_dir = (round(y_dir_o/min(abs(y_dir_o))));

x_limit = round(rotated_corners(1,1));
x_start = round(rotated_corners(1,4));

y_limit = 200-round(rotated_corners(2,3));
y_start = 200-round(rotated_corners(2,4));

else
x_dir = x_dir_o;
y_dir = y_dir_o;
x_limit = round(rotated_corners(1,3));
x_start = round(rotated_corners(1,1));

y_limit = round(rotated_corners(2,1));
y_start = round(rotated_corners(2,2));

end


while y_start > y_limit
    j = y_start;
    for i = x_start:y_dir(1):x_limit
        mapmat(j,i)= 0.5;
        j = j - y_dir(2);
    end
    x_start = x_start+x_dir(1);
    y_start = y_start-x_dir(2);
    x_limit = x_limit+x_dir(1);
end


figure(2)
map = occupancyMap(mapmat,5);
show(map);hold on
line(rotated_coor(1,:)*0.2,rotated_coor(2,:)*0.2,'color',c,'LineWidth',1);hold on
plot(search_origin(1)*0.2,search_origin(2)*0.2,'ob','linewidth',1);hold on
%search_map = occupancyMap(searchmatrix,5);


xLeft = ul_corner(1)*0.2;
yBottom = ul_corner(2)*0.2;

width = (lr_corner(1)-ul_corner(1))*0.2;
hight = abs((ul_corner(2)-lr_corner(2))*0.2);
%corr = RotateArea(x0,width,hight,45);


%show(map); hold on
%plot(search_origin(1)*0.2,search_origin(2)*0.2,'ob','linewidth',2);hold on
%rectangle('Position', [xLeft, yBottom, width, hight], 'EdgeColor','b', 'LineWidth', 1);

%figure(2)
%show(search_map)









%% Construct Vehicle Safe Area
function [safe_zone_min_x,safe_zone_max_x,safe_zone_min_y,safe_zone_max_y,safe_zone] = safe_area(config,x0,map,predicted_x,safe_zone,mpciter)
    max_search_x = 2;
    max_search_y = 2;
    safe_zone_boarders = [];
    safe_zone_min_x = [];
    safe_zone_min_y = [];
    safe_zone_max_x = [];
    safe_zone_max_y = [];
    mapmat = round(occupancyMatrix(map));

    for k = 1:config.N
    if ~isempty(predicted_x)
    max_search_x = 2;
    max_search_y = 2;
    predicted_x_s = predicted_x(k,:,mpciter);
    robot_curr_x = predicted_x_s(1);
    robot_curr_y = predicted_x_s(2);
    robot_curr_th = predicted_x_s(3);
    index_in_map = local2grid(map,[robot_curr_x, robot_curr_y]);
    else
        robot_curr_x = x0(1);
        robot_curr_y = x0(2);
        robot_curr_th = x0(3);
        index_in_map = local2grid(map,[robot_curr_x, robot_curr_y]);
    end
    
    min_y_search_index = max(index_in_map(1)-max_search_y*map.Resolution,1);
    max_y_search_index = min(index_in_map(1)+max_search_y*map.Resolution,size(mapmat,1));
    min_x_search_index = max(index_in_map(2)-max_search_x*map.Resolution,1);
    max_x_search_index = min(index_in_map(2)+max_search_x*map.Resolution,size(mapmat,2));

    searchmatrix = mapmat(min_y_search_index:max_y_search_index,...
                          min_x_search_index:max_x_search_index);
    
    search_area_min_x = index_in_map(2)-max_search_x*map.Resolution;
    search_area_max_y = index_in_map(1)-max_search_y*map.Resolution;

    [leftcorner, rightcorner] = getMaxSearchArea(searchmatrix, max_search_x*map.Resolution,max_search_y*map.Resolution);
    Upper_left_corner_local = grid2local(map,[search_area_max_y+leftcorner(2),search_area_min_x+leftcorner(1)]);
    Lower_right_corner_local = grid2local(map,[search_area_max_y+rightcorner(2),search_area_min_x+rightcorner(1)]);
    


    safe_zone_min_x1 = Upper_left_corner_local(1)+0.2;
    safe_zone_max_x1 = Lower_right_corner_local(1)-0.2;
    safe_zone_min_y1 = Lower_right_corner_local(2)+0.2;
    safe_zone_max_y1 = Upper_left_corner_local(2)-0.2; 

    safe_zone_min_x = [safe_zone_min_x ;safe_zone_min_x1];
    safe_zone_min_y = [safe_zone_min_y ;safe_zone_min_y1];
    safe_zone_max_x = [safe_zone_max_x ;safe_zone_max_x1];
    safe_zone_max_y = [safe_zone_max_y ;safe_zone_max_y1];
    
    safe_zone = [safe_zone; [safe_zone_min_x1 safe_zone_max_x1 safe_zone_min_y1 safe_zone_max_y1]];
    
    end
end


%% Search Neighbour function
function obst_list = search_near(point, x_search, y_search, map)
obst_list = [];
mapmat = round(getOccupancy(map));
point = local2grid(map,[point(1), point(2)]);
for i = round(point(2)-x_search*map.Resolution):round(point(2)+x_search*map.Resolution)           % Search the whole Area
    for j = round(point(1)-y_search*map.Resolution):round(point(1)+y_search*map.Resolution)
        if mapmat(j,i) == 1                                                                     % if you found an obstacle
            obst_pose = grid2local(map,[j,i]);
            obst_list = [obst_list ; obst_pose];
        end
    end
end
end