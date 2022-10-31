clear, clc


mapmat = zeros(300,300);

x_obs1 = [10,10.2];
y_obs1 = [0,15];

for i = x_obs1(1)*10:x_obs1(2)*10
    for j = y_obs1(1)*10+1:y_obs1(2)*10
        mapmat(i,j) = 1;
    end
end

x_obs1 = [10,20];
y_obs1 = [15,15.2];

for i = x_obs1(1)*10:x_obs1(2)*10
    for j = y_obs1(1)*10:y_obs1(2)*10
        mapmat(i,j) = 1;
    end
end

x_obs1 = [10,10.2];
y_obs1 = [22,30];

for i = x_obs1(1)*10:x_obs1(2)*10
    for j = y_obs1(1)*10:y_obs1(2)*10
        mapmat(i,j) = 1;
    end
end

x_obs1 = [10,20];
y_obs1 = [15,15.2];

for i = x_obs1(1)*10:x_obs1(2)*10
    for j = y_obs1(1)*10:y_obs1(2)*10
        mapmat(i,j) = 1;
    end
end

x_obs1 = [10,20];
y_obs1 = [22,22.2];

for i = x_obs1(1)*10:x_obs1(2)*10
    for j = y_obs1(1)*10:y_obs1(2)*10
        mapmat(i,j) = 1;
    end
end

x_obs1 = [20,20.2];
y_obs1 = [0.1,15.2];

for i = x_obs1(1)*10:x_obs1(2)*10
    for j = y_obs1(1)*10:y_obs1(2)*10
        mapmat(i,j) = 1;
    end
end

x_obs1 = [20,20.2];
y_obs1 = [22,30];

for i = x_obs1(1)*10:x_obs1(2)*10
    for j = y_obs1(1)*10:y_obs1(2)*10
        mapmat(i,j) = 1;
    end
end

x_obs1 = [26,26.2];
y_obs1 = [0.1,30];

for i = x_obs1(1)*10:x_obs1(2)*10
    for j = y_obs1(1)*10:y_obs1(2)*10
        mapmat(i,j) = 1;
    end
end

x_obs1 = [6,7];
y_obs1 = [10,13];

for i = x_obs1(1)*10:x_obs1(2)*10
    for j = y_obs1(1)*10:y_obs1(2)*10
        mapmat(i,j) = 1;
    end
end

x_obs1 = [23,26];
y_obs1 = [8,9];

for i = x_obs1(1)*10:x_obs1(2)*10
    for j = y_obs1(1)*10:y_obs1(2)*10
        mapmat(i,j) = 1;
    end
end

x_obs1 = [4,4.2];
y_obs1 = [0.1,30];

for i = x_obs1(1)*10:x_obs1(2)*10
    for j = y_obs1(1)*10:y_obs1(2)*10
        mapmat(i,j) = 1;
    end
end


x0 = [8.5;10];
map1 = load('Maps/omni_map1.mat');
map = occupancyMap(map1.imageOccupancy,5);
show(map); hold on
plot(x0(1),x0(2),'ob','linewidth',2);hold on
 % Update the local safe Area
    max_search_x = 2;
    max_search_y = 3;
    safe_zone_min_x = x0(1)-max_search_x;
    safe_zone_max_x = x0(1)+max_search_x;
    safe_zone_min_y = x0(2)-max_search_y;
    safe_zone_max_y = x0(2)+max_search_y;
    safe_zone_boarders = [];
    safe_zone_final = [];
    robot_curr_x = x0(1);
    robot_curr_y = x0(2);
    index_in_map = local2grid(map,[robot_curr_x, robot_curr_y]);
    
    searchmatrix = map1.imageOccupancy(index_in_map(1)-max_search_y*map.Resolution:index_in_map(1)+max_search_y*map.Resolution,...
                         index_in_map(2)-max_search_x*map.Resolution:index_in_map(2)+max_search_x*map.Resolution);

    search_area_min_x = index_in_map(2)-max_search_x*map.Resolution;
    search_area_max_y = index_in_map(1)-max_search_y*map.Resolution;

    [leftcorner, rightcorner] = getMaxSearchArea(searchmatrix);
    left_corner_local = grid2local(map,[search_area_max_y+leftcorner(1)-1,search_area_min_x+leftcorner(2)-1]);
    right_corner_local = grid2local(map,[search_area_max_y+rightcorner(1)-1,search_area_min_x+rightcorner(2)-1]);

    xLeft = left_corner_local(1);
    yBottom = right_corner_local(2);
    
    width = right_corner_local(1)-left_corner_local(1);
    hight = left_corner_local(2)-right_corner_local(2);
    %corr = RotateArea(x0,width,hight,45);
    rectangle('Position', [xLeft, yBottom, width, hight], 'EdgeColor','b', 'LineWidth', 1);
%% Generate Maplab matrix from map
clear;clc    
image = imread('Maps/map2.png');
    imageNorm = double(squeeze(image(:,:,1)))/255;
    imageOccupancy = 1 - imageNorm;
    ware_house_map = occupancyMap(imageOccupancy,5);
    show(ware_house_map);
