function b = isObstacle(pos, obstacles)

n = size(obstacles, 1);

b = false;

for i=1:n
    if pos(1) == obstacles(i,1) && pos(2) == obstacles(i,2)
        b = true;
        break;
    end
end

end