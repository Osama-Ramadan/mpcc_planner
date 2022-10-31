clear, clc
load exampleMaps.mat
map = binaryOccupancyMap(complexMap);
show(map)

mapLocal = binaryOccupancyMap(complexMap(end-20:end,1:20));
show(mapLocal)

path = [5 2
        8 2
        8 8
        30 8];
show(map)
hold on
plot(path(:,1),path(:,2))
hold off

for i = 1:length(path)-1
    moveAmount = (path(i+1,:)-path(i,:))/map.Resolution;
    for j = 1:abs(moveAmount(1)+moveAmount(2))
        moveValue = sign(moveAmount).*map.Resolution;
        move(mapLocal,moveValue, ...
            "MoveType","relative","SyncWith",map)
 
        show(mapLocal)
        drawnow limitrate
        pause(0.2)
    end
end