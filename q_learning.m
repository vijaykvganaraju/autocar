function q_table = q_learning(mapsize, target, obstacles, numIter)
    q_table = zeros([mapsize, 4]);
    lr = 0.1;
    gamma = 0.9;
    
    for i = 1 : numIter

        pos = [randi(mapsize(1)), randi(mapsize(2))];
        
        while ~isObstacle(pos, obstacles) && ~isTarget(pos, target)
            action = randi(4);
            % going left
            if action == 1
                if pos(1) ~= 1
                    next_pos = [pos(1) - 1, pos(2)];
                else
                    continue
                end
            % going right
            elseif action == 2
                if pos(1) ~= mapsize(1)
                    next_pos = [pos(1) + 1, pos(2)];
                else
                    continue
                end
            % going up
            elseif action == 3
                if pos(2) ~= mapsize(2)
                    next_pos = [pos(1), pos(2) + 1];
                else
                    continue
                end
            % going down
            elseif action == 4
                if pos(2) ~= 1
                    next_pos = [pos(1), pos(2) - 1];
                else
                    continue
                end
            end
            
            % q(s, a) = q(s, a) + lr * [ reward + discount * max(q(s', a')) - q(s, a) ]
            q_table(pos(1), pos(2), action) = q_table(pos(1), pos(2), action) + lr * (environment(pos, action, mapsize, target, obstacles) + gamma * max(q_table(next_pos(1), next_pos(2), :)) - q_table(pos(1), pos(2), action));
            pos = next_pos;
        end
    
    end
end
