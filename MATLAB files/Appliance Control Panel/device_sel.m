function varargout = device_sel(varargin)
% DEVICE_SEL M-file for device_sel.fig
%      DEVICE_SEL, by itself, creates a new DEVICE_SEL or raises the existing
%      singleton*.
%
%      H = DEVICE_SEL returns the handle to a new DEVICE_SEL or the handle to
%      the existing singleton*.
%
%      DEVICE_SEL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DEVICE_SEL.M with the given input arguments.
%
%      DEVICE_SEL('Property','Value',...) creates a new DEVICE_SEL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before device_sel_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to device_sel_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help device_sel

% Last Modified by GUIDE v2.5 11-Sep-2009 17:06:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @device_sel_OpeningFcn, ...
                   'gui_OutputFcn',  @device_sel_OutputFcn, ...
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


% --- Executes just before device_sel is made visible.
function device_sel_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to device_sel (see VARARGIN)

% Choose default command line output for device_sel
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes device_sel wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = device_sel_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
figure(tv)

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
figure(ac)
