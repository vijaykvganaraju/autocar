classdef TwoWheeledRobot    
    properties
        R;
        L;
        wL;
        wR;
        w;
    end
    
    methods
        function robot = TwoWheeledRobot(R, L, wL, wR, w)
            %TWOWHEELEDROBOT Construct an instance of this class
            %   Detailed explanation goes here
            robot.R = R;
            robot.L = L;
            robot.wL = wL;
            robot.wR = wR;
            robot.w = w;
        end
        function disp_info(robot)
           fprintf('Value of R = %f\n', robot.R);
           fprintf('Value of L = %f\n', robot.L);
           fprintf('Value of wL = %f\n', robot.wL);
           fprintf('Value of wR = %f\n', robot.wR);
           fprintf('Value of w = %f\n', robot.w);
        end
        function [finalState, states] = goingStraight(robot, currentState, T, delta)
            full_steps = floor(T / delta);
%             fprintf("Time = %f, delta = %f and Full steps = %f\n", T, delta, full_steps);
            state = currentState;
            states = currentState;
            
            for step = 1 : full_steps
                nextState = zeros(1, 3);
                cos_theta = cos(state(3));
                sin_theta = sin(state(3));
                if cos_theta > -1e-10 && cos_theta < 1e-10
                    cos_theta = 0;
                end
                if sin_theta > -1e-10 && sin_theta < 1e-10
                    sin_theta = 0;
                end
                nextState(1) = state(1) + robot.R * robot.w * cos_theta * delta;
                nextState(2) = state(2) + robot.R * robot.w * sin_theta * delta;
                nextState(3) = state(3);
%                 disp(nextState)
                states = [states; nextState];
                state = nextState;
            end
            
            last_stepsize = T - floor(T / delta) * delta;
%             fprintf("Last stepsize = %f, Time = %f and Full steps = %f\n", last_stepsize, T, full_steps);
            if last_stepsize > 0
                nextState = zeros(1, 3);
                cos_theta = cos(state(3));
                sin_theta = sin(state(3));
                if cos_theta > -1e-10 && cos_theta < 1e-10
                    cos_theta = 0;
                end
                if sin_theta > -1e-10 && sin_theta < 1e-10
                    sin_theta = 0;
                end
                nextState(1) = state(1) + robot.R * robot.w * cos_theta * last_stepsize;
                nextState(2) = state(2) + robot.R * robot.w * sin_theta * last_stepsize;
                nextState(3) = state(3);
                
                states = [states; nextState];
                state = nextState;
            end
            
            finalState = state;
        end
        
        function [finalState, states] = turningLeft(robot, currentState, T, delta)
            full_steps = floor(T / delta);
%             fprintf("Time = %f, delta = %f and Full steps = %f\n", T, delta, full_steps);
            state = currentState;
            states = currentState;
            for step = 1 : full_steps
                nextState = zeros(1, 3);
                cos_theta = cos(state(3));
                sin_theta = sin(state(3));
                cos_theta_d = cos(state(3) + robot.R / robot.L * robot.wR * delta);
                sin_theta_d = sin(state(3) + robot.R / robot.L * robot.wR * delta);
                
                if cos_theta > -1e-10 && cos_theta < 1e-10
                    cos_theta = 0;
                end
                if sin_theta > -1e-10 && sin_theta < 1e-10
                    sin_theta = 0;
                end
                if cos_theta_d > -1e-10 && cos_theta_d < 1e-10
                    cos_theta = 0;
                end
                if sin_theta_d > -1e-10 && sin_theta_d < 1e-10
                    sin_theta = 0;
                end
                nextState(1) = state(1) + robot.L / 2 * (sin_theta_d - sin_theta);
                nextState(2) = state(2) + robot.L / 2 * (cos_theta - cos_theta_d);
                nextState(3) = state(3) + (robot.R / robot.L) * robot.wR * delta;
                
                states = [states; nextState];
                state = nextState;
            end
            
            last_stepsize = T - floor(T / delta) * delta;
            
            if last_stepsize > 0
%                 fprintf("Last stepsize = %f, Time = %f and Full steps = %f\n", last_stepsize, T, full_steps);
                nextState = zeros(1, 3);
                cos_theta = cos(state(3));
                sin_theta = sin(state(3));
                cos_theta_d = cos(state(3) + robot.R / robot.L * robot.wR * last_stepsize);
                sin_theta_d = sin(state(3) + robot.R / robot.L * robot.wR * last_stepsize);
                
                if cos_theta > -1e-10 && cos_theta < 1e-10
                    cos_theta = 0;
                end
                if sin_theta > -1e-10 && sin_theta < 1e-10
                    sin_theta = 0;
                end
                if cos_theta_d > -1e-10 && cos_theta_d < 1e-10
                    cos_theta = 0;
                end
                if sin_theta_d > -1e-10 && sin_theta_d < 1e-10
                    sin_theta = 0;
                end
                nextState(1) = state(1) + robot.L / 2 * (sin_theta_d - sin_theta);
                nextState(2) = state(2) + robot.L / 2 * (cos_theta - cos_theta_d);
                nextState(3) = state(3) + (robot.R / robot.L) * robot.wR * delta;
                
                states = [states; nextState];
                state = nextState;
            end
            
            finalState = state;
        end
        
        function [finalState, states] = turningRight(robot, currentState, T, delta)
            full_steps = floor(T / delta);
%             fprintf("Time = %f, delta = %f and Full steps = %f", full_steps);
            state = currentState;
            states = currentState;
            
            for step = 1 : full_steps
                nextState = zeros(1, 3);
                cos_theta = cos(state(3));
                sin_theta = sin(state(3));
                cos_theta_d = cos(state(3) - robot.R / robot.L * robot.wL * delta);
                sin_theta_d = sin(state(3) - robot.R / robot.L * robot.wL * delta);
                
                if cos_theta > -1e-10 && cos_theta < 1e-10
                    cos_theta = 0;
                end
                if sin_theta > -1e-10 && sin_theta < 1e-10
                    sin_theta = 0;
                end
                if cos_theta_d > -1e-10 && cos_theta_d < 1e-10
                    cos_theta = 0;
                end
                if sin_theta_d > -1e-10 && sin_theta_d < 1e-10
                    sin_theta = 0;
                end
                nextState(1) = state(1) + robot.L / 2 * (sin_theta - sin_theta_d);
                nextState(2) = state(2) + robot.L / 2 * (cos_theta_d - cos_theta);
                nextState(3) = state(3) - (robot.R/robot.L) * robot.wL * delta;
                
                states = [states; nextState];
                state = nextState;
            end
            
            last_stepsize = T - floor(T / delta) * delta;
            
            if last_stepsize > 0
                nextState = zeros(1, 3);
                cos_theta = cos(state(3));
                sin_theta = sin(state(3));
                cos_theta_d = cos(state(3) + robot.R / robot.L * robot.wL * last_stepsize);
                sin_theta_d = sin(state(3) + robot.R / robot.L * robot.wL * last_stepsize);
                
                if cos_theta > -1e-10 && cos_theta < 1e-10
                    cos_theta = 0;
                end
                if sin_theta > -1e-10 && sin_theta < 1e-10
                    sin_theta = 0;
                end
                if cos_theta_d > -1e-10 && cos_theta_d < 1e-10
                    cos_theta = 0;
                end
                if sin_theta_d > -1e-10 && sin_theta_d < 1e-10
                    sin_theta = 0;
                end
                nextState(1) = state(1) + robot.L / 2 * (sin_theta - sin_theta_d);
                nextState(2) = state(2) + robot.L / 2 * (cos_theta_d - cos_theta);
                nextState(3) = state(3) - (robot.R/robot.L) * robot.wL * last_stepsize;
                
                states = [states; nextState];
                state = nextState;
            end
            
            finalState = state;
        end
        
        function [states] = simulate(robot, initialState, modes, durations, delta)
            finalState = initialState;
            states = [];
        
            for ins_no = 1 : length(modes)
%                 fprintf("Direction = %c and Time = %f\n", directions(ins_no), times(ins_no));
                if modes(ins_no) == 'S' || modes(ins_no) == 's'
                    [finalState, currentInstructionState] = robot.goingStraight(finalState, durations(ins_no), delta);
                elseif modes(ins_no) == 'L' || modes(ins_no) == 'l'
                    [finalState, currentInstructionState] = robot.turningLeft(finalState, durations(ins_no), delta);
                elseif modes(ins_no) == 'R' || modes(ins_no) == 'r'
                    [finalState, currentInstructionState] = robot.turningRight(finalState, durations(ins_no), delta);
                end
                states = [states; currentInstructionState];
            end
            
        end
        
        function DeltaTheta = computeDeltaTheta(robot, currentState, targetPosition)

            x0 = currentState(1);
            y0 = currentState(2);
            theta0 = currentState(3);

            xT = targetPosition(1);
            yT = targetPosition(2);

            heading = [cos(theta0), sin(theta0)];
            target = [xT - x0, yT - y0];
            % Numerator of cross product of heading and target
            hxt_numerator = heading(1) * target(2) - heading(2) * target(1);

            if hxt_numerator >= -1e-12 && hxt_numerator <= 1e-12
                % straight
                % Numerator of dot product of heading and target
                hot_numerator = heading(1) * target(1) + heading(2) * target(2);
                
                if hot_numerator > 0
                    % same direction or straight
                    DeltaTheta = 0;
                else
                    % opposite direction or u-turn
                    xC = x0 - (robot.L / 2) * cos(theta0 - pi / 2);
                    yC = y0 - (robot.L / 2) * sin(theta0 - pi / 2);

                    DeltaTheta = 2 * pi - 2 * acos(robot.L / (2 * (sqrt((xT - xC)^2 + (yT - yC)^2))));
                end

            elseif hxt_numerator > 0
                % left
                xC = x0 - (robot.L / 2) * cos(theta0 - pi / 2);
                yC = y0 - (robot.L / 2) * sin(theta0 - pi / 2);

                beta = acos(robot.L / (2 * (sqrt((xT - xC)^2 + (yT - yC)^2))));
                alpha = acos(((x0 - xC) * (xT - xC) + (y0 - yC) * (yT -yC)) / (sqrt((x0 - xC)^2 + (y0 - yC)^2) * sqrt((xT - xC)^2 + (yT - yC)^2)));
                alpha_range_check = (x0 - xC) * (yT - yC) - (xT -xC) * (y0 - yC);
        %         fprintf("Beta = %f, alpha_range = %f", beta, alpha_range_check);

                if alpha_range_check < 0
                    % less than pi
                    DeltaTheta = 2 * pi - alpha - beta;
                else
                    % greater than pi
                    DeltaTheta = alpha - beta;
                end
            else
                % right
                xC = x0 + (robot.L / 2) * cos(theta0 - pi / 2);
                yC = y0 + (robot.L / 2) * sin(theta0 - pi / 2);

                beta = acos(robot.L / (2 * (sqrt((xT - xC)^2 + (yT - yC)^2))));

                alpha = acos(((x0 - xC) * (xT - xC) + (y0 - yC) * (yT -yC)) / (sqrt((x0 - xC)^2 + (y0 - yC)^2) * sqrt((xT - xC)^2 + (yT - yC)^2)));
                alpha_range_check = (x0 - xC) * (yT - yC) - (xT -xC) * (y0 - yC);
        %         fprintf("Beta = %f, alpha_range = %f", beta, alpha_range_check);

                if alpha_range_check < 0
                    % less than pi
                    DeltaTheta = alpha - beta;
                else
                    % greater than pi
                    DeltaTheta = 2 * pi - alpha - beta;
                end
                DeltaTheta = -DeltaTheta;
            end
        end

        
        function [modes, times, angles] = p2pReach(robot, currentState, targetPosition)
            deltaTheta = robot.computeDeltaTheta(currentState, targetPosition);
            delta = 0.005;
            x0 = currentState(1);
            y0 = currentState(2); 
%             theta0 = currentState(3);

            xT = targetPosition(1);
            yT = targetPosition(2);
            
            modes = [];
            times = [];
            angles = [];

            if deltaTheta == 0
                time = sqrt((xT - x0) ^ 2 + (yT - y0) ^ 2) / (robot.R * robot.w);
                modes = [modes, 'S'];
                times = [times, time];
                angles = [angles; currentState(3) + deltaTheta];
            elseif deltaTheta > 0
                time = abs(deltaTheta) / (robot.R * robot.w);
                [finalState, ~] = robot.turningLeft(currentState, time, delta);
                
                modes = [modes, 'L'];
                times = [times, time];
                angles = [angles; currentState(3) + deltaTheta];

                xL = finalState(1);
                yL = finalState(2);
                time = sqrt((xT - xL) ^ 2 + (yT - yL) ^ 2) / (robot.R * robot.w);
                
                modes = [modes, 'S'];
                times = [times, time];
                angles = [angles; currentState(3) + deltaTheta];

            elseif deltaTheta < 0
                time = abs(deltaTheta) / (robot.R * robot.w);
                [finalState, ~] = robot.turningRight(currentState, time, delta);
                
                modes = [modes, 'R'];
                times = [times, time];
                angles = [angles; currentState(3) + deltaTheta];

                xL = finalState(1);
                yL = finalState(2);
                time = sqrt((xT - xL) ^ 2 + (yT - yL) ^ 2) / (robot.R * robot.w);
                
                modes = [modes, 'S'];
                times = [times, time];
                angles = [angles; currentState(3) + deltaTheta];
            end
        
        end
        
    end
end

