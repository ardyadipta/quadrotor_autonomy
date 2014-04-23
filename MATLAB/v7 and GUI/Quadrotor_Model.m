% MRSD Robot Autonomy Project
% Trajectory following
% Author: Fan Zhang
classdef Quadrotor_Model < handle
    properties
        % -- Position in world frame
        x = 0;
        y = 0;
        z = 0;
        % -- velocity in world frame
        d_x = 0;
        d_y = 0;
        d_z = 0;
        % -- Euler angles
        phi = 0;    % roll
        theta = 0;  % pitch
        psi = 0;    % yaw
        % -- Angluar velocity in body frame
        p = 0;
        q = 0;
        r = 0;
        % -- rotor speed
        omega = [0 0 0 0]; % rotor [1..4]
        omega_h = 0;
        leave_off = 0;
        kF = 6.11e-8;   % N/(rpm)^2
        kM = 1.5e-9;    % Nm/(rpm)^2
        km = 20;        % s-1
        % -- Other
        g = 9.8;
        I = 0.005;
        L = 0.17933;
        m = 0.6;
    end
    
    methods
        function obj = setPosition(obj, x_new, y_new, z_new)
            obj.x = x_new;
            obj.y = y_new;
            obj.z = z_new;
        end
        function obj = initialize(obj)
            obj.omega_h = sqrt(obj.m * obj.g / 4 / obj.kF);
            obj.omega = repmat(obj.omega_h, 1, 4);
        end
    end
end