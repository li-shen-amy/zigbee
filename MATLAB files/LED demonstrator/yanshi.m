function varargout = yanshi(varargin)
% YANSHI M-file for yanshi.fig
%      YANSHI, by itself, creates a new YANSHI or raises the existing
%      singleton*.
%
%      H = YANSHI returns the handle to a new YANSHI or the handle to
%      the existing singleton*.
%
%      YANSHI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in YANSHI.M with the given input arguments.
%
%      YANSHI('Property','Value',...) creates a new YANSHI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before yanshi_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to yanshi_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help yanshi

% Last Modified by GUIDE v2.5 24-Sep-2009 15:29:47

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @yanshi_OpeningFcn, ...
                   'gui_OutputFcn',  @yanshi_OutputFcn, ...
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


% --- Executes just before yanshi is made visible.
function yanshi_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to yanshi (see VARARGIN)

% Choose default command line output for yanshi
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes yanshi wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = yanshi_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function senddata(x)   % send data: x through serial port com1 
s=serial('com1');        
fopen(s)             %Open serial port com1 
s.baudrate=115200;     % Set baudrate 
fwrite(s,x);          % Send data  

function readdata(x)   % read data: x through serial port com1 
s=serial('com1');        
fopen(s)             %Open serial port com1 
s.baudrate=115200;     % Set baudrate 
fread(s,x);          % Read data  


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Mystring=get(hObject,'String');
if(Mystring=='ON')
senddata(3);
set(hObject,'String','OFF');
guidata(hObject,handles);
else senddata(7);
set(hObject,'String','ON');
guidata(hObject,handles);
end

% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Mystring=get(hObject,'String');
if(Mystring=='ON')
senddata(4);
set(hObject,'String','OFF');
guidata(hObject,handles);
else senddata(8);
set(hObject,'String','ON');
guidata(hObject,handles);
end

% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Mystring=get(hObject,'String');
if(Mystring=='ON')
senddata(1);
set(hObject,'String','OFF');
guidata(hObject,handles);
else senddata(5);
set(hObject,'String','ON');
guidata(hObject,handles);
end


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Mystring=get(hObject,'String');
if(Mystring=='ON')
senddata(2);
set(hObject,'String','OFF');
guidata(hObject,handles);
else senddata(6);
set(hObject,'String','ON');
guidata(hObject,handles);
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
readdata(x);
msg_string=strcat(msg_string,x);
set(hObject,'String',msg_string);
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
