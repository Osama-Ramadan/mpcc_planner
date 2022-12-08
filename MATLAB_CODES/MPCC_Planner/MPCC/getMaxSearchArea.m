function [best_ll,best_ur] = getMaxSearchArea(matrix)
    rows = size(matrix,1);
    max_area_p = 0;
    max_height_p = 0;
    max_right_p = 0;
    max_left_p = 0;
    for r = round(rows/2):rows
        [max_area,bar_index,max_height,max_right,max_left] = get_histogramArea(matrix(1:r,:),rows);
       
        if max_area> max_area_p
            max_area_p = max_area;
            max_right_p = max_right;
            max_left_p = max_left;
            max_height_p = max_height;
            max_row = r;
        end
    end

    best_ll = [max_row-max_height_p+1,max_left_p];
    best_ur = [max_row,max_right_p];

end

function [max_area,bar_index,max_height,max_right,max_left] = get_histogramArea(rows,n_total_rows)
    cols = size(rows,2);
    n_rows = size(rows,1);
    histogram = zeros(1,cols);

    for i = 1:size(rows,1)
        row = rows(i,:);
        for col = 1:cols
            if ~(row(col)==0)
                histogram(col)=0;
            else
                histogram(col) = 1 + histogram(col);
            end
        end
    end

    [max_area,bar_index,max_height,max_right,max_left] = largestRectangleAreaHistogram(histogram,n_rows,n_total_rows);
end

function [max_area,bar_index,max_height,max_right,max_left] = largestRectangleAreaHistogram(heights,n_rows,n_total_rows)
    n = length(heights);
    p_horiz = round(n/2);
    p_vert = round(n_total_rows/2);
    left = [];
    right = [];
    stack = [];
    % Getting the left element
    for i = 1:n
        if isempty(stack)
            left(i) = 1;
        else
            while ~isempty(stack) && heights(stack(end))>=heights(i)
                    stack(end) = [];
            end
            
            if isempty(stack)
                left(i) = 1;
            else
                left(i) = stack(end)+1;
            end

        end
        stack = [stack ; i];
    end
    stack = [];
    
    % Getting the right element
    for i = n:-1:1
        if isempty(stack)
            right(i) = n;
        else
            while ~isempty(stack) && heights(stack(end))>=heights(i)
                    stack(end) = [];
            end
            
            if isempty(stack)
                right(i) = n;
            else
                right(i) = stack(end)-1;
            
            end
        end
        stack = [stack ; i];
    end

    %calculating the area
    max_area=0;
    bar_index=0;
    max_height=0;
    max_right=0;
    max_left = 0;
    for i = 1:n
        height = heights(i);
        width = right(i)-left(i);
        area = height*width;

        if area > max_area && (left(i)<p_horiz && right(i)>p_horiz) &&(n_rows>=p_vert && (n_total_rows-heights(i)<(p_vert-2)))
            max_area = area;
            max_height = height;
            max_right = right(i);
            max_left = left(i);
            bar_index = i;

        end
    end
end
