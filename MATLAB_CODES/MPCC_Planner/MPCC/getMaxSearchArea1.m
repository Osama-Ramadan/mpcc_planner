function [best_ll,best_ur] = getMaxSearchArea1(matrix,x_r,y_r)
    c = zeros(size(matrix,1),1);
    N = size(matrix,2);
    M = size(matrix,1);
    best_ll = [0;0];
    best_ur = [-1;-1];

    for ll_x = N:-1:1
        c = update_cache(ll_x,c,matrix);
        for ll_y = 1:M
            [ur_x,ur_y] = grow_ones(ll_x,ll_y,matrix,c);
            if ((area(ll_x,ll_y,ur_x,ur_y) > (area(best_ll(1),best_ll(2),best_ur(1),best_ur(2)))) && (x_r>ll_x && x_r< ur_x) && (y_r>ll_y && y_r<ur_y))
                best_ll = [ll_x;ll_y];
                best_ur = [ur_x,ur_y];
            end
        end
    end
end

function c = update_cache(ll_x,c,matrix) 
    for y = 1:size(matrix,1)
        if matrix(y,ll_x)==0
            c(y) = c(y)+1;
        else
            c(y) = 0;
        end
    end
end

function [ur_x,ur_y] = grow_ones(ll_x,ll_y,matrix,c)
    ur_x = ll_x-1;
    ur_y = ll_y-1;
    x_max = size(matrix,2);
    y = ll_y;
    while(y <= size(matrix,1) && matrix(y,ll_x)==0)
        x = min(ll_x+c(y),x_max);
        x_max = x;
        if area(ll_x,ll_y,x,y) > area(ll_x,ll_y,ur_x,ur_y)
            ur_x = x;
            ur_y = y;
        end
        y = y+1;
    end
end

function A = area(ll_x,ll_y,ur_x,ur_y)
    if ll_x > ur_x || ll_y > ur_y
        A = 0;
        return;
    else
        A = (ur_x-ll_x)*(ur_y-ll_y);
    end
end