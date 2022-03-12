function plotMap(map, grid_on_off)

n = size(map);

if grid_on_off
    for i = 1:n(2)+1
       line([0.5,n(1)+0.5],[i-0.5,i-0.5]);
    end

    for i = 1:n(1)+1
       line([i-0.5,i-0.5],[0.5,n(2)+0.5]);
    end

    hold on;
end


for i = 1:n(1)
    for j = 1:n(2)
        if map(i,j) == 0
            plotCircle(i,j,'r');
            hold on;
        end
    end
end

axis([-0.5,n(1)+1.5,-0.5,n(2)+1.5]);
axis equal;

end