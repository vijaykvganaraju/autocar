clear; close all; clc;

robot = TwoWheeledRobot(0.2, 1, pi, pi, pi);

origin = [
    0, 0, (3/4) * pi;
    -1, 2, pi;
    -2, -1, (5/4) * pi;
    -5, -1, (1/4) * pi;
    0, 0, pi / 2];

destination = [
    -3, 3;
    2, -3;
    2, 3;
    -4, -1;
    1, -2];

if length(origin) == length(destination)
    for test_case = 1 : length(origin)
        initial_state = origin(test_case, :);
        final_coordinate = destination(test_case, :);
        [modes, durations] = robot.p2pReach(initial_state, final_coordinate);
        figure,
        allStates = robot.simulate(initial_state, modes, durations, 0.01);
        plot(allStates(:, 1), allStates(:, 2));hold on;
        plot(initial_state(1), initial_state(2), 'ro');
        plot(final_coordinate(1), final_coordinate(2), 'ro');
        hold off;
        title_test_case = "Test Case " + test_case;
        title(title_test_case);
        axis equal; 
    end
else
    disp('origin and destination size do not match!')
end
