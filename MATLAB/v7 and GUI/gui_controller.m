function varargout = gui_controller(varargin)
% GUI_CONTROLLER MATLAB code for gui_controller.fig
%      GUI_CONTROLLER, by itself, creates a new GUI_CONTROLLER or raises the existing
%      singleton*.
%
%      H = GUI_CONTROLLER returns the handle to a new GUI_CONTROLLER or the handle to
%      the existing singleton*.
%
%      GUI_CONTROLLER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_CONTROLLER.M with the given input arguments.
%
%      GUI_CONTROLLER('Property','Value',...) creates a new GUI_CONTROLLER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_controller_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_controller_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_controller

% Last Modified by GUIDE v2.5 22-Apr-2014 19:48:17

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_controller_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_controller_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before gui_controller is made visible.
function gui_controller_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_controller (see VARARGIN)

% Choose default command line output for gui_controller
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui_controller wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%DEFAULT VALUES
kp_phi_set = 500;
kd_phi_set = 1000;
kp_theta_set = 500;
kd_theta_set = 1000;
kp_psi_set = 1;
kd_psi_set = 1;
% Stiff hover controller
kp_stiff_x_set = 1;
kp_stiff_y_set = 1;
kp_stiff_z_set = 1;

kd_stiff_x_set = 5; 
kd_stiff_y_set = 5; 
kd_stiff_z_set = 1; 
ki_stiff_set = [0.01 0.01 0.01];

set(handles.kp_phi, 'Value', kp_phi_set);
set(handles.kd_phi_s, 'Value', kd_phi_set);

set(handles.kp_theta_s, 'Value', kp_phi_set);
set(handles.kd_theta_s, 'Value', kd_phi_set);

set(handles.kp_psi_s, 'Value', kp_phi_set);
set(handles.kd_psi_s, 'Value', kd_phi_set);

set(handles.kp_stiff_x, 'Value', kp_stiff_x_set);
set(handles.kd_stiff_x, 'Value', kd_stiff_x_set);

set(handles.kp_stiff_y, 'Value', kp_stiff_y_set);
set(handles.kd_stiff_y, 'Value', kd_stiff_y_set);

set(handles.kp_stiff_z, 'Value', kp_stiff_z_set);
set(handles.kd_stiff_z, 'Value', kd_stiff_z_set);



set(handles.kp_phi_value,'String', strcat('', num2str(kp_phi_set,3)) );
set(handles.kd_phi_value,'String', strcat('', num2str(kd_phi_set,3)) );
set(handles.kp_theta_value,'String', strcat('', num2str(kp_theta_set,3)) );
set(handles.kd_theta_value,'String', strcat('', num2str(kd_theta_set,3)) );
set(handles.kp_psi_value,'String', strcat('', num2str(kp_psi_set,3)) );
set(handles.kd_psi_value,'String', strcat('', num2str(kd_psi_set,3)) );
set(handles.kp_stiff_x_value,'String', strcat('', num2str(kp_stiff_x_set,3)) );
set(handles.kp_stiff_y_value,'String', strcat('', num2str(kp_stiff_y_set,3)) );
set(handles.kp_stiff_z_value,'String', strcat('', num2str(kp_stiff_z_set,3)) );
set(handles.kd_stiff_x_value,'String', strcat('', num2str(kd_stiff_x_set,3)) );
set(handles.kd_stiff_y_value,'String', strcat('', num2str(kd_stiff_y_set,3)) );
set(handles.kd_stiff_z_value,'String', strcat('', num2str(kd_stiff_z_set,3)) );





% --- Outputs from this function are returned to the command line.
function varargout = gui_controller_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function kp_phi_Callback(hObject, eventdata, handles)
% hObject    handle to kp_phi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

kp_phi_set = get(handles.kp_phi, 'Value');
set(handles.kp_phi_value,'String', strcat('', num2str(kp_phi_set,3)) );

%plot(handles.axes1,x,y);


% --- Executes during object creation, after setting all properties.
function kp_phi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kp_phi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function kd_phi_s_Callback(hObject, eventdata, handles)
kd_phi_set = get(handles.kd_phi_s, 'Value');
set(handles.kd_phi_value,'String', strcat('', num2str(kd_phi_set,3)) );


% --- Executes during object creation, after setting all properties.
function kd_phi_s_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function kp_theta_s_Callback(hObject, eventdata, handles)
kp_theta_set = get(handles.kp_theta_s, 'Value');
set(handles.kp_theta_value,'String', strcat('', num2str(kp_theta_set,3)) );


% --- Executes during object creation, after setting all properties.
function kp_theta_s_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function kd_theta_s_Callback(hObject, eventdata, handles)
kd_theta_set = get(handles.kd_theta_s, 'Value');
set(handles.kd_theta_value,'String', strcat('', num2str(kd_theta_set,3)) );


% --- Executes during object creation, after setting all properties.
function kd_theta_s_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function kp_psi_s_Callback(hObject, eventdata, handles)
% hObject    handle to kp_psi_s (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kp_psi_set = get(handles.kp_psi_s, 'Value');
set(handles.kp_psi_value,'String', strcat('', num2str(kp_psi_set,3)) );


% --- Executes during object creation, after setting all properties.
function kp_psi_s_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kp_psi_s (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function kd_psi_s_Callback(hObject, eventdata, handles)
% hObject    handle to kd_psi_s (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kd_psi_set = get(handles.kd_psi_s, 'Value');
set(handles.kd_psi_value,'String', strcat('', num2str(kd_psi_set,3)) );


% --- Executes during object creation, after setting all properties.
function kd_psi_s_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kd_psi_s (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function kp_stiff_x_Callback(hObject, eventdata, handles)
% hObject    handle to kp_stiff_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kp_stiff_x_set = get(handles.kp_stiff_x, 'Value') ;
set(handles.kp_stiff_x_value,'String', strcat('', num2str(kp_stiff_x_set,3)) );


% --- Executes during object creation, after setting all properties.
function kp_stiff_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kp_stiff_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function kp_stiff_y_Callback(hObject, eventdata, handles)
% hObject    handle to kp_stiff_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kp_stiff_y_set = get(handles.kp_stiff_y, 'Value') ;
set(handles.kp_stiff_y_value,'String', strcat('', num2str(kp_stiff_y_set,3)) );


% --- Executes during object creation, after setting all properties.
function kp_stiff_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kp_stiff_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function kp_stiff_z_Callback(hObject, eventdata, handles)
% hObject    handle to kp_stiff_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kp_stiff_z_set = get(handles.kp_stiff_z, 'Value') ;
set(handles.kp_stiff_z_value,'String', strcat('', num2str(kp_stiff_z_set,3)) );


% --- Executes during object creation, after setting all properties.
function kp_stiff_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kp_stiff_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function kd_stiff_x_Callback(hObject, eventdata, handles)
% hObject    handle to kd_stiff_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kd_stiff_x_set = get(handles.kd_stiff_x, 'Value') ;
set(handles.kd_stiff_x_value,'String', strcat('', num2str(kd_stiff_x_set,3)) );


% --- Executes during object creation, after setting all properties.
function kd_stiff_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kd_stiff_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function kd_stiff_y_Callback(hObject, eventdata, handles)
% hObject    handle to kd_stiff_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kd_stiff_y_set = get(handles.kd_stiff_y, 'Value') ;
set(handles.kd_stiff_y_value,'String', strcat('', num2str(kd_stiff_y_set,3)) );


% --- Executes during object creation, after setting all properties.
function kd_stiff_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kd_stiff_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function kd_stiff_z_Callback(hObject, eventdata, handles)
% hObject    handle to kd_stiff_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kd_stiff_z_set = get(handles.kd_stiff_z, 'Value') ;
set(handles.kd_stiff_z_value,'String', strcat('', num2str(kd_stiff_z_set,3)) );


% --- Executes during object creation, after setting all properties.
function kd_stiff_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kd_stiff_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in simulate_button.
function simulate_button_Callback(hObject, eventdata, handles)
% hObject    handle to simulate_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla(handles.axes1,'reset');
cla(handles.axes2,'reset');

kp_phi_set = get(handles.kp_phi, 'Value');
kd_phi_set = get(handles.kd_phi_s, 'Value');
kp_theta_set = get(handles.kp_theta_s, 'Value');
kd_theta_set = get(handles.kd_theta_s, 'Value');
kp_psi_set = get(handles.kp_psi_s, 'Value');
kd_psi_set = get(handles.kd_psi_s, 'Value');
kp_stiff_x_set = get(handles.kp_stiff_x, 'Value') ;
kp_stiff_y_set = get(handles.kp_stiff_y, 'Value') ;
kp_stiff_z_set = get(handles.kp_stiff_z, 'Value') ;
kd_stiff_x_set = get(handles.kd_stiff_x, 'Value') ;
kd_stiff_y_set = get(handles.kd_stiff_y, 'Value') ;
kd_stiff_z_set = get(handles.kd_stiff_z, 'Value') ;
kp_stiff_set = [kp_stiff_x_set kp_stiff_y_set kp_stiff_z_set];
kd_stiff_set = [kd_stiff_x_set kd_stiff_y_set kd_stiff_z_set];
ki_stiff_set = [0.01 0.01 0.01];

%% Demo for how to use simulation
%% Initialization
model = Quadrotor_Model;
model.initialize;
model.setPosition(0, 0, 0);
controller = Quadrotor_Controller;

controller.setControllerCoef(kp_phi_set, kd_phi_set, kp_theta_set,kd_theta_set, kp_psi_set, kd_psi_set, kp_stiff_set, kd_stiff_set, ki_stiff_set);

controller.setDeltaT(0.01);
controller.setConvergeErrorThreshold(0.05);
viz = Quadrotor_Visualizer;
viz.setProperties(5, 0.6);
% simulation parameters
viz_3d = 0;
viz_error = 0;
viz_interval = 3;

%% Waypoints following by Regulation (Hover control)
controller.setControlMode(2);
% Two waypoint
% waypoints = [0 0 0.5 0; 1 1 1 0];
% Randomly choose 5 waypoints
waypoints = [0 0 1 0; 0.5 0 1.5 0;]; % 0.5 0.5 2 0; 1 0.5 1.5 0; 1 1 2 0];
% Following A line
% waypoints = [linspace(0.5, 2, 50)' linspace(0.5, 2, 50)' repmat([0.5 0], 50, 1)];
% Following A Circle

% start waypoint following
[state_data] = quadrotor_track_waypoints(model, controller, waypoints, ...
    viz, viz_3d, viz_error, viz_interval);
% visualize results in 3D
% figure;hold on;grid on;
% desired straight line trajectory
axes(handles.axes1);
hold on
plot3(handles.axes1, waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 'r');
% real trajectory
plot3(handles.axes1, state_data(:, 1), state_data(:, 2), state_data(:, 3), 'b');
hold off

%% Waypoints following by Trajectory Following
controller.setControlMode(2);
% one waypoint
waypoints = [0 0 1 0; 0.5 0 1.5 0;];% 2 1 3 0; 3 3 3 0; 4 5 2 0];
% Randomly choose 5 waypoints
% waypoints = [0 0 1 0; 0.5 0 1.5 0; 0.5 0.5 2 0; 1 0.5 1.5 0; 1 1 2 0];
% Following A line
% waypoints = [linspace(0.5, 2, 50)' linspace(0.5, 2, 50)' repmat([0.5 0], 50, 1)];
% Time for each segment of trajectory
timepoints = [0 1.0];% 2.0 3.0 4.0];
% Calculate Polynomial Function Trajectory using Quadratic Programming

% Simple Version
% t0 = 0;
% t1 = t0 + time_segments(1);
% coefficient for x, y, z, yaw
% format : [cx3 cx2 cx1 cx0 cy3 cy2 cy1 cy0 cz3 cz2 cz1 cz0 cyaw3 cyaw2 cyaw1 cyaw0]
% for x, y: 0s at 0, 1.0s at 1.0
% trajectory_coef = [1 0 0 0 1 0 0 0 0.5 0 0 0.5 0 0 0 0];
time_factor = 1;
number_order = 5;
mu_r = 1;
mu_yaw = 1;
trajectory_coef = getTrajectoryCoef(waypoints, timepoints, ...
    number_order, mu_r, mu_yaw, time_factor);
% trajectory_coef = trajectory_coef';
% start trajectory following
[state_data] = quadrotor_track_trajectory(model, controller, waypoints, ...
    trajectory_coef, timepoints, viz, viz_3d, viz_error, viz_interval);

% visualize results in 3D
% figure;hold on;grid on;
% desired straight line trajectory
trajectory = [];
trajectory_x = [];
trajectory_y = [];
trajectory_z = [];
for seg_id = 2 : length(timepoints)
    t = timepoints(seg_id - 1) : controller.dt : timepoints(seg_id);
    t = [repmat(1, 1, size(t, 2)); t; t .^ 2; t .^ 3; t .^ 4; t .^ 5];
    coefficient = reshape(trajectory_coef(:, seg_id - 1), 4, number_order + 1);
    if length(trajectory_x) == 0
        trajectory_x = (coefficient(1, :) * t)';
        trajectory_y = (coefficient(2, :) * t)';
        trajectory_z = (coefficient(3, :) * t)';
        continue;
    end
    trajectory_x = [trajectory_x' coefficient(1, :) * t]';
    trajectory_y = [trajectory_y' coefficient(2, :) * t]';
    trajectory_z = [trajectory_z' coefficient(3, :) * t]';
end
trajectory = [trajectory_x trajectory_y trajectory_z];
% plot3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 'g');
% plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'r');
% % real trajectory
% plot3(state_data(:, 1), state_data(:, 2), state_data(:, 3), 'b');

% for GUI
axes(handles.axes2);
hold on
plot3(handles.axes2, waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 'g');

plot3(handles.axes2, trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'r');
%real trajectory
plot3(handles.axes2, state_data(:, 1), state_data(:, 2), state_data(:, 3), 'b');
hold off


% --- Executes on button press in clear_button.
function clear_button_Callback(hObject, eventdata, handles)
% hObject    handle to clear_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.axes1,'reset');
cla(handles.axes2,'reset');
