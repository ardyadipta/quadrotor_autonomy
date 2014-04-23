function [state_data] = quadrotor_track_trajectory(model, controller, ...
    waypoints, trajectory_coef, timepoints, ...
    viz, viz_3d, viz_error, viz_interval)
%% Quadrotor Trajectroy Following Simulation
if nargin < 2
    viz_3d = false;
    viz_error = true;
else
    if nargin < 3
        viz_error = true;
    end
end

state_data = [];
error_data = [];
pos_id = 1;
%% Fly to the first way point until converge
last_error_seq = [];
controller.setDesiredPosition(waypoints(pos_id, 1), waypoints(pos_id, 2), waypoints(pos_id, 3));
while true
    controller.update_state(model);
    x = [model.x model.y model.z ...
        model.d_x model.d_y model.d_z ...
        model.phi model.theta model.psi ...
        model.p model.q model.r ...
        model.omega];
    u = controller.des_omega';
    [t, y] = ode45(@quadrotor_state_update,[0 controller.dt], x, [], controller.des_omega);
    new_x = y(end, :);
    state_data = [state_data; new_x];
    model.x = new_x(1);model.y = new_x(2);model.z = new_x(3);
    model.d_x = new_x(4);model.d_y = new_x(5);model.d_z = new_x(6);
    model.phi = new_x(7);model.theta = new_x(8);model.psi = new_x(9);
    model.p = new_x(10);model.q = new_x(11);model.r = new_x(12);
    model.omega = new_x(13 : 16);
    
    if length(last_error_seq) == 100
        last_error_seq = [last_error_seq(2:end) sqrt(sum((x(1 : 3) - controller.pos_goal(1 : 3)) .^ 2))];
    else
        last_error_seq = [last_error_seq sqrt(sum((x(1 : 3) - controller.pos_goal(1 : 3)) .^ 2))];
    end
    %if sqrt(sum((x(1 : 3) - controller.pos_goal(1 : 3)) .^ 2)) < 0.01
    if mean(last_error_seq) < 0.01
        disp('Reached first waypoint, start to follow trajectory.');
        break
    end
end

%% Follow Trajectory
seg_id = 1;
now_time = 0;
% generate coefficient matrx for first segment
coefficient = reshape(trajectory_coef(:, seg_id), 4, 6)';
while seg_id <= length(timepoints)
    % controller.set
    tic;
    while 1 == 1
        now_time = now_time + controller.dt;
        desired_state = getDesiredStateFromTrajectory(coefficient, now_time);
        controller.setDesiredPosition(desired_state(1), desired_state(2), desired_state(3));
        controller.setDesiredLinearVelocity(desired_state(4), desired_state(5), desired_state(6));
        controller.setDesiredLinearAcceleration(desired_state(7), desired_state(8), desired_state(9));
        controller.setDesiredEulerAngle(desired_state(10), desired_state(11), desired_state(12));
        controller.setDesiredAngularVelocityBodyFrame(desired_state(13), desired_state(14), desired_state(15));
        controller.update_state(model);
        x = [model.x model.y model.z ...
            model.d_x model.d_y model.d_z ...
            model.phi model.theta model.psi ...
            model.p model.q model.r ...
            model.omega];
        u = controller.des_omega';
        %% update state
        [t, y] = ode45(@quadrotor_state_update,[0 controller.dt], x, [], controller.des_omega);
        new_x = y(end, :);
        state_data = [state_data; new_x];
        
        model.x = new_x(1);model.y = new_x(2);model.z = new_x(3);
        model.d_x = new_x(4);model.d_y = new_x(5);model.d_z = new_x(6);
        model.phi = new_x(7);model.theta = new_x(8);model.psi = new_x(9);
        model.p = new_x(10);model.q = new_x(11);model.r = new_x(12);
        model.omega = new_x(13 : 16);

        %% visualize error
        error_data = [error_data; new_x(1) - controller.pos_goal(1), new_x(2) - controller.pos_goal(2), new_x(3) - controller.pos_goal(3), ...
            new_x(4) - controller.velocity_goal(1), new_x(5) - controller.velocity_goal(2), new_x(6) - controller.velocity_goal(3), ...
            new_x(7) - controller.des_phi, new_x(8) - controller.des_theta, new_x(9) - controller.des_psi, ...
            new_x(10) - controller.des_p, new_x(11) - controller.des_q, new_x(12) - controller.des_r];
        
        if viz_interval > toc
            viz_interval = toc;
            continue;
        end
        if viz_error
            % position, speed, euler angle
            figure(4);
            subplot(2, 1, 1);
            plot(1 : size(error_data, 1), error_data(:, 1) , 1 : size(error_data, 1), error_data(:, 2), 1 : size(error_data, 1), error_data(:, 3), ...
                1 : size(error_data, 1), error_data(:, 4), 1 : size(error_data, 1), error_data(:, 5), 1 : size(error_data, 1), error_data(:, 6));
            legend('x', 'y', 'z', 'x_d', 'y_d', 'z_d', 'Location', 'NorthWest');
            subplot(2, 1, 2);
            plot(1 : size(error_data, 1), error_data(:, 7), 1 : size(error_data, 1), error_data(:, 8), 1 : size(error_data, 1), error_data(:, 9), ...
                1 : size(error_data, 1), error_data(:, 10), 1 : size(error_data, 1), error_data(:, 11), 1 : size(error_data, 1), error_data(:, 12));
            legend('phi', 'theta', 'psi', 'p', 'q', 'r', 'Location', 'NorthWest');
            
            % % print state
            % disp(sprintf('Des x: %.3f y: %.3f z: %.3f', controller.pos_goal(1), controller.pos_goal(2), controller.pos_goal(3)));
            % disp(sprintf('Now x: %.3f y: %.3f z: %.3f', model.x, model.y, model.z));
            % disp(sprintf('Now dx: %.3f dy: %.3f dz: %.3f', model.d_x, model.d_y, model.d_z));
            % disp(sprintf('Des phi: %.3f theta: %.3f psi: %.3f', controller.des_phi, controller.des_theta, controller.des_psi));
            % disp(sprintf('Now phi: %.3f theta: %.3f psi: %.3f', model.phi, model.theta, model.psi));
            % disp(sprintf('Des p: %.3f q: %.3f r: %.3f', controller.des_p, controller.des_q, controller.des_r));
            % disp(sprintf('Now p: %.3f q: %.3f r: %.3f', model.p, model.q, model.r));
        end
        %% visualize quadrotor
        if viz_3d
            figure(5);
            % visualize quadrotor
            viz.setPosition([model.x model.y model.z]);
            viz.setEulerAngle(model.phi, model.theta, model.psi);
            viz.show(gcf);
        end
        
        % Check switch to next segment
        if now_time >= 2%time_segments(seg_id)
            disp(sprintf('Reached %d waypoint', seg_id));
            % generate coefficient matrx for next segment
            seg_id = seg_id + 1;
            if seg_id < size(waypoints, 1)
                coefficient = reshape(trajectory_coef(:, seg_id), 4, 6)';
            end
            break
        end
        %pause(0.001);
    end
end



