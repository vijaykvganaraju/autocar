classdef Process
    properties
        name;
        rows;
        cols;
        env;
        initial;
        target;
        obstacles;
        q_table;
        path;
        smoothed_path;
        valid_env = false;
        valid_initial = false;
        valid_target = false;
    end
    
    methods
        function process = Process(name)
            process.name = name;
            fprintf('[INFO from: %s]: Initiating process %s...\n', process.name, process.name);
        end
        
        function display_info(process)
            fprintf(' Process Name: %s\n', process.name);
            fprintf(' Rows in env: %d\n', process.rows);
            fprintf(' Columns in env: %d\n', process.cols);
            fprintf(' Initial position: %d, %d\n', process.initial(1), process.initial(2));
            fprintf(' Target position: %d, %d\n', process.target(1), process.target(2));
            fprintf(' Environment valid? %d\n', process.valid_env);
            fprintf(' Initial position valid? %d\n', process.valid_initial);
            fprintf(' Target position valid? %d\n', process.valid_target);
        end
        
        function load_info(process, info)
            fprintf('[INFO from: %s]: %s\n', process.name, info);
        end
        
        function err(process, info)
            fprintf('[ERROR from: %s]: %s\n', process.name, info);
        end
        
        function process = validate_env(process, rows, cols)
            if isreal(rows) && isreal(cols) && floor(rows) == rows && floor(cols) == cols && rows > 0 && cols > 0 && rows ~= inf && cols ~= inf
                process.rows = rows;
                process.cols = cols;
                process.env = ones(rows, cols);
                process.valid_env = true;
            else
                process.err('Invalid Rows or Columns value. Please enter POSITIVE INTEGERS less than inf.')
            end
        end
        
        function process = validate_pos(process, pos, pos_name)
            if pos(1) >= 1 && pos(2) >= 1 && pos(1) <= process.rows && pos(2) <= process.cols
                if strcmp(pos_name, 'initial')
                    process.valid_initial = true;
                    process.initial = pos;
                    return;
                elseif strcmp(pos_name, 'target')
                    process.valid_target = true;
                    process.target = pos;
                    return;
                elseif strcmp(pos_name, 'obstacle')
                    if ~(isequal(pos, process.initial) || isequal(pos, process.target))
                        process.obstacles = [process.obstacles; pos];
                        return;
                    else
                        process.err('Obstacle value must not be equal to Initital value or Target value');
                    end
                end
            else
                err_info = ['Invalid' pos_name ' value. Range of Target Value x must be in [0, ' num2str(process.rows) ') and y must be in [0, ' num2str(process.cols) ')'];
                process.err(err_info);
            end 
        end
        
        function process = create_map(process)
            numOfObstacles = size(process.obstacles, 1);

            for i=1:numOfObstacles
                process.env(process.obstacles(i,1), process.obstacles(i,2)) = 0;
            end
            env = process.env;
        end
        
        function process = q_learn(process)
            process.load_info('Started learning...')
            process.q_table = q_learning([process.rows, process.cols], process.target, process.obstacles, 1000);
            process.load_info('Started learning... Done')
        end
        
        function process = create_path(process, smoothing)
            current_pos = process.initial;

            while current_pos(1) ~= process.target(1) || current_pos(2) ~= process.target(2)

               action = 1;
               quality = process.q_table(current_pos(1), current_pos(2), action);

               if quality < process.q_table(current_pos(1), current_pos(2), 2)
                   action = 2;
                   quality = process.q_table(current_pos(1), current_pos(2), 2);
               end

               if quality < process.q_table(current_pos(1), current_pos(2), 3)
                   action = 3;
                   quality = process.q_table(current_pos(1), current_pos(2), 3);
               end

               if quality < process.q_table(current_pos(1), current_pos(2), 4)
                   action = 4;
               end

               switch action
                   case 1
                       current_pos(1) = current_pos(1) - 1;
                   case 2
                       current_pos(1) = current_pos(1) + 1;
                   case 3
                       current_pos(2) = current_pos(2) + 1;
                   case 4
                       current_pos(2) = current_pos(2) - 1;
               end
               process.path = [process.path; current_pos];
            end
            for i = 1 : smoothing : size(process.path, 1)
                process.smoothed_path = [process.smoothed_path; process.path(i, :)];
            end
            if isequal(process.smoothed_path(end, :), process.path(end, :))
                process.smoothed_path = [process.smoothed_path; process.path(end, :)];
            end
        end
    end
end