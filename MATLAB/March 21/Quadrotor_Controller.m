classdef Quadrotor_Controller < handle
    properties
        % Goal position and velocity and acceleration
        pos_goal = [0 0 0];
        velocity_goal = [0 0 0];
        acc_goal = [0 0 0];
        % Desired position vector
        %des_pos = [0 0 0];
        % Desired velocity vector
        des_d_pos = [0 0 0];
        % Desired acceleration vector
        des_dd_x = 0;
        des_dd_y = 0;
        des_dd_z = 0;
        % Desired Angle
        des_psi = 0;
        des_phi = 0;
        des_theta = 0;
        % Desired Angular Velocity
        des_p = 0;
        des_q = 0;
        des_r = 0;
        % Desired increment rotor speed
        d_omega_phi = 0;
        d_omega_theta = 0;
        d_omega_psi = 0;
        d_omega_F = 0;
        % Desired rotor speed
        des_omega = [0 0 0 0];
        % Parameters
        g = 10;
        dt = 0.01;
        % Controller Mode 
        % 1: Attitude 2: Hover 3: Trajectory
        control_mode = 3;
        % Attitude Controller
        kp_phi = 500;
        kd_phi = 1000;
        kp_theta = 500;
        kd_theta = 1000;
        kp_psi = 1;
        kd_psi = 1;

        % Stiff hover controller
        kp_stiff = [1 1 1];
        kd_stiff = [5 5 1];
        ki_stiff = [0.01 0.01 0.01];
        % Soft hover controller
        kp_soft = [1 1 1];
        kd_soft = [1 1 1];
        ki_soft = [1 1 1];
        % Trajectory controller
        kp_tc = [1 1 1];
        kd_tc = [1 1 1];
        % PID integral part
        err_last = [0 0 0];
        err_prop = [0 0 0];
        err_int = [0 0 0];
        err_der = [0 0 0];
    end
    
    methods
        function obj = setControlMode(obj, mode)
            obj.control_mode = mode;
        end
        function obj = setDeltaT(obj, dt)
            obj.dt = dt;
        end
        function obj = setDesiredPosition(obj, new_x, new_y, new_z)
            obj.pos_goal = [new_x new_y new_z];
            obj.err_last = [0 0 0];
            obj.err_prop = [0 0 0];
            obj.err_int = [0 0 0];
            obj.err_der = [0 0 0];
        end
        function obj = setDesiredLinearVelocity(obj, new_d_x, new_d_y, new_d_z)
            obj.velocity_goal = [new_d_x new_d_y new_d_z];
        end
        function obj = setDesiredAngularVelocityBodyFrame(obj, new_p, new_q, new_r)
            obj.des_p = new_p;
            obj.des_q = new_q;
            obj.des_r = new_r;
        end

        function obj = setDesiredEulerAngle(obj, new_phi, new_theta, new_psi)
            obj.des_psi = new_psi; %z-axis 
            obj.des_phi = new_phi; %x -axis 
            obj.des_theta = new_theta; %y-axis
        end
        function obj = update_state(obj, model)
            switch obj.control_mode
                case 1
                    obj.update_attitude_control(model);
                case 2
                    obj.update_hover_control(model);
                case 3
                    obj.update_trajectory_control(model);
            end
        end
        function obj = update_attitude_control(obj, model)
            obj.d_omega_phi = obj.kp_phi * (obj.des_phi - model.phi) + obj.kd_phi * (obj.des_p - model.p);
            obj.d_omega_theta = obj.kp_theta * (obj.des_theta - model.theta) + obj.kd_theta * (obj.des_q - model.q);
            obj.d_omega_psi = obj.kp_psi * (obj.des_psi - model.psi) + obj.kd_psi * (obj.des_r - model.r);
            obj.des_omega = [1 0 -1 1;1 1 0 -1; 1 0 1 1; 1 -1 0 -1] ...
                * [model.omega_h + obj.d_omega_F; obj.d_omega_phi; obj.d_omega_theta; obj.d_omega_psi];
        end
        function obj = update_hover_control(obj, model)
            obj.err_prop = obj.pos_goal - [model.x model.y model.z];
            obj.err_der = obj.velocity_goal - [model.d_x model.d_y model.d_z]; %(obj.err_prop - obj.err_last) / obj.dt
            obj.err_int = obj.err_int + (obj.err_prop + obj.err_last) * obj.dt / 2; %
            obj.err_last = obj.err_prop;
            
            desired_acc = obj.kd_stiff .* obj.err_der + obj.kp_stiff .* obj.err_prop + obj.ki_stiff .* obj.err_int;
            obj.des_dd_x = desired_acc(1);
            obj.des_dd_y = desired_acc(2);
            obj.des_dd_z = desired_acc(3);
            
            obj.des_phi = 1 / obj.g * (obj.des_dd_x * sin(obj.des_psi) - obj.des_dd_y * cos(obj.des_psi));
            obj.des_theta = 1 / obj.g * (obj.des_dd_x * cos(obj.des_psi) + obj.des_dd_y * sin(obj.des_psi));
            obj.d_omega_F = model.m / 8 / model.kF / model.omega_h * obj.des_dd_z;
            % pass to attitude control
            obj.update_attitude_control(model);
        end
        function obj = update_trajectory_control(obj, model)
            % position and velocity error
            ep = obj.pos_goal - [model.x model.y model.z]; % don't know how to calculate normal vector and binormal vector
            ev = obj.velocity_goal - [model.d_x, model.d_y, model.d_z];
            %ev = [0 0 0];
            % Calculate desired acceleration 
            obj.des_dd_x = obj.kp_tc(1) * ep(1) + obj.kd_tc(1) * ev(1) + obj.acc_goal(1);
            obj.des_dd_y = obj.kp_tc(2) * ep(2) + obj.kd_tc(2) * ev(2) + obj.acc_goal(2);
            obj.des_dd_z = obj.kp_tc(3) * ep(3) + obj.kd_tc(3) * ev(3) + obj.acc_goal(3);
            
            % pass to hover controller
            obj.update_hover_control(model);
        end        
    end
end