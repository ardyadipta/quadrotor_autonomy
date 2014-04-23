%% Quadrotor Visualization
classdef Quadrotor_Visualizer < handle
    properties
        length = 0;
        mass = 0;
        prop_radius = 0;
        center_pos = [0 0 0];
        phi = 0;
        theta = 0;
        psi = 0;
    end
    methods
        function obj = setProperties(obj, length, mass)
            obj.length = length;
            obj.mass = mass;
        end
        function obj = setPosition(obj, pos)
            obj.center_pos = pos;
        end
        function obj = setEulerAngle(obj, phi, theta, psi)
            obj.phi = phi;
            obj.theta = theta;
            obj.psi = psi;
        end
        function obj = show(obj, fig_handle)
            figure(fig_handle);
            hold off;grid on;
            rotation = obj.getBodyToWorldRotation;
            rotor1 = rotation * (obj.center_pos + [obj.length 0 0])';
            rotor2 = rotation * (obj.center_pos + [0 obj.length 0])';
            rotor3 = rotation * (obj.center_pos + [-obj.length 0 0])';
            rotor4 = rotation * (obj.center_pos + [0 -obj.length 0])';
            plot3([rotor1(1) rotor3(1)], [rotor1(2) rotor3(2)], [rotor1(3) rotor3(3)],'LineWidth', 5);
            hold on;
            plot3([rotor2(1) rotor4(1)], [rotor2(2) rotor4(2)], [rotor2(3) rotor4(3)],'LineWidth', 5);
            axis([-15 15 -15 15 0 10]);
            
        end
        function R = getBodyToWorldRotation(obj)
            R = [cos(obj.psi) * cos(obj.theta) - sin(obj.phi) * sin(obj.psi) * sin(obj.theta) ...
                -cos(obj.phi) * sin(obj.psi) ...
                cos(obj.psi) * sin(obj.theta) + cos(obj.theta) * sin(obj.phi) * sin(obj.psi);...
                cos(obj.theta) * sin(obj.psi) + cos(obj.psi) * sin(obj.phi) * sin(obj.theta) ...
                cos(obj.phi) * cos(obj.psi) ...
                sin(obj.psi) * sin(obj.theta) - cos(obj.psi) * cos(obj.theta) * sin(obj.phi);...
                -cos(obj.phi) * sin(obj.theta) ...
                sin(obj.phi) ...
                cos(obj.phi) * cos(obj.theta)];
        end
    end
end