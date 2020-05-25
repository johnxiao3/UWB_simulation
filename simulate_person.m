function varargout = simulate_person(varargin)
% SIMULATE_PERSON MATLAB code for simulate_person.fig
%      SIMULATE_PERSON, by itself, creates a new SIMULATE_PERSON or raises the existing
%      singleton*.
%
%      H = SIMULATE_PERSON returns the handle to a new SIMULATE_PERSON or the handle to
%      the existing singleton*.
%
%      SIMULATE_PERSON('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMULATE_PERSON.M with the given input arguments.
%
%      SIMULATE_PERSON('Property','Value',...) creates a new SIMULATE_PERSON or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before simulate_person_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to simulate_person_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help simulate_person

% Last Modified by GUIDE v2.5 25-May-2020 10:33:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @simulate_person_OpeningFcn, ...
                   'gui_OutputFcn',  @simulate_person_OutputFcn, ...
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


% --- Executes just before simulate_person is made visible.
function simulate_person_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to simulate_person (see VARARGIN)

% Choose default command line output for simulate_person
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes simulate_person wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = simulate_person_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

axes(handles.main);
box on;
xlim([-18 18]);ylim([-18 18]);
axis equal;

handles.m_location = [5,5];
handles.xo = [0.15,-0.15,0];
handles.yo = [0.0,0.0,0.15];
handles.No = size(handles.xo,2);
for i = 1:handles.No
    handles.d(i) = norm([handles.xo(i) handles.yo(i)]-handles.m_location);  
    rand_err = normrnd(0,0.018);
    handles.d_m(i) = handles.d(i)+rand_err;
end
handles.Kalman_No = 9;
handles.Kalman_No=handles.Kalman_No+1;
if handles.Kalman_No == 10
    handles.Kalman_No = 0;
    handles.P = handles.d_m+1;
    handles.d_m_k = handles.d_m;
end
handles.tag_m_k = handles.m_location;

handles.t = timer('StartDelay', 1, 'Period', 0.4,...
          'ExecutionMode', 'fixedRate','TimerFcn', {@TimerFnc,hObject});
start(handles.t)

guidata(hObject,handles);

function TimerFnc(hObject, EventData,hFigure)
handles = guidata(hFigure);
for i = 1:handles.No
    handles.d(i) = norm([handles.xo(i) handles.yo(i)]-handles.m_location);  
    rand_err = normrnd(0,0.018);
    handles.d_m(i) = handles.d(i)+rand_err;
end

R_k = [1 1 1];
handles.Kalman_No=handles.Kalman_No+1;
if handles.Kalman_No == 10
    handles.Kalman_No = 0;
    handles.P = handles.d_m+1;
    handles.d_m_k = handles.d_m;
end
handles.d_mbar = 1*handles.d_m_k;
handles.Pbar = handles.P;
handles.K=handles.Pbar./(handles.Pbar+R_k);
handles.d_m_k=handles.d_mbar+handles.K.*(handles.d_m-handles.d_mbar);
handles.P = (1-handles.K).*(handles.Pbar);

d_m_k = handles.d_m_k;
xo = handles.xo;
yo = handles.yo;
% method 3 LLS with Kalman
gamma1 = d_m_k(2)^2-d_m_k(1)^2-(xo(2)^2-xo(1)^2+yo(2)^2-yo(1)^2);
gamma2 = d_m_k(3)^2-d_m_k(1)^2-(xo(3)^2-xo(1)^2+yo(3)^2-yo(1)^2);
A = 2*[xo(1)-xo(2) yo(1)-yo(2)
   xo(1)-xo(3) yo(1)-yo(3)];
handles.tag_m_k = inv(A)*[gamma1;gamma2];

guidata(hFigure,handles);
update_plot(handles);

% --- Executes on key press with focus on figure1 and none of its controls.
function figure1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
if strcmp(eventdata.Key,'uparrow')
    handles.m_location(2) = handles.m_location(2) + 0.2;
elseif strcmp(eventdata.Key,'downarrow')
    handles.m_location(2) = handles.m_location(2) - 0.2;
elseif strcmp(eventdata.Key,'leftarrow')
    handles.m_location(1) = handles.m_location(1) - 0.2;
elseif strcmp(eventdata.Key,'rightarrow')
    handles.m_location(1) = handles.m_location(1) + 0.2;
end
% handles = update_plot(handles);
guidata(hObject,handles);

function handles = update_plot(handles)
axes(handles.main);
cla;
syms x; syms y;
fimplicit(@(x,y) (x-handles.m_location(1)).^2+...
    (y-handles.m_location(2)).^2-0.8.^2,'r');
hold on;
xlim([-18 18]);hold on;
ylim([-18 18]);
axis square;
title([num2str(handles.m_location(1)),'-',num2str(handles.m_location(2))]);
plot(handles.xo,handles.yo,'.b','MarkerSize',4);
plot(handles.tag_m_k(1),handles.tag_m_k(2),'.k','MarkerSize',20);



% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
delete(hObject);
stop(handles.t);
delete(handles.t);