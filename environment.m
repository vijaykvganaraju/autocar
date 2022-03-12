function reward = environment(pos, action, mapsize, target, obstacles)

    switch action
        case 1      % going left
            if pos(1) == 1
                reward = -100;
            else
                next_pos = [pos(1)-1, pos(2)];

                if isObstacle(next_pos, obstacles)
                    reward = -100;
                elseif isTarget(next_pos, target)
                    reward = 100;
                else
                    reward = 0;
                end
            end

        case 2      % going right
            if pos(1) == mapsize(1)
                reward = -100;
            else
                next_pos = [pos(1)+1, pos(2)];

                if isObstacle(next_pos, obstacles)
                    reward = -100;
                elseif isTarget(next_pos, target)
                    reward = 100;
                else
                    reward = 0;
                end
            end

        case 3      % going up
            if pos(2) == mapsize(2)
                reward = -100;
            else
                next_pos = [pos(1), pos(2)+1];

                if isObstacle(next_pos, obstacles)
                    reward = -100;
                elseif isTarget(next_pos, target)
                    reward = 100;
                else
                    reward = 0;
                end
            end

        case 4      % going down
            if pos(2) == 1
                reward = -100;
            else
                next_pos = [pos(1), pos(2)-1];

                if isObstacle(next_pos, obstacles)
                    reward = -100;
                elseif isTarget(next_pos, target)
                    reward = 100;
                else
                    reward = 0;
                end
            end
    end

end