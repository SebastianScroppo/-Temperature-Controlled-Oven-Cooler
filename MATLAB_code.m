function varargout = myscope(varargin)
gui_Singleton = 1; 
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @myscope_OpeningFcn, ...
                   'gui_OutputFcn',  @myscope_OutputFcn, ...
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
%-----------------------------------------------------------------------
% --- Executes just before myscope is made visible.
function myscope_OpeningFcn(hObject, eventdata, handles, varargin)
global comport
global RUN
global NPOINTS

RUN = 0;
NPOINTS = 200;
comport = serial('COM7', 'BaudRate', 115200);
fopen(comport);
% Choose default command line output for myscope
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
%-----------------------------------------------------------------------
function myscope_OutputFcn(hObject, eventdata, handles, varargin)
%-----------------------------------------------------------------------
function Quit_Button_Callback(hObject, eventdata, handles)
global comport
global RUN
RUN = 0;
fclose(comport);
clear comport
if ~isempty(instrfind)
fclose(instrfind);
delete(instrfind);
end
% use close to terminate your program
% use quit to terminate MATLAB
close

% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
T=get(hObject,'value');
set(handles.text4,'string',T)

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


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



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%----------------------------------------------------------------------
function Run_Button_Callback(hObject, eventdata, handles)
global comport
global NPOINTS
global RUN
points=0
%T= 40;
%P= 1;
%I= 1;
%D= 1;

% TPID = [T P I D];
% get parameters from GUI:
T=str2num(get(handles.text4,'string'));
P=str2num(get(handles.edit2,'string'));
I=str2num(get(handles.edit3,'string'));
D=str2num(get(handles.edit4,'string'));

%disp(P);
%send parameters to IAR:
fprintf(comport, '%s', char(T));
fprintf(comport, '%s', char(P));
fprintf(comport, '%s', char(I));
fprintf(comport, '%s', char(D));

%disp(TPID)

if RUN == 0
  RUN = 1;
  % change the string on the button to STOP
  set(handles.Run_Button,'string','STOP')
  points = points + NPOINTS;
  fopen('test1.txt', 'w+');          %create text file to store data to plot 
  while RUN == 1 % ADD YOUR CODE HERE. 
    % send a single character prompt to the MCU
    
    %fprintf(comport,'A');
    % fetch data as single 8-bit bytes
    d = fread(comport, NPOINTS, 'uint8')      %recieve digital voltage readings
    
    Temp=-0.000009*d.^3+0.0043*d.^2-0.8951*d+74.794;   %convert digital voltage readings to temperature
    graph = fopen('test1.txt', 'a+');
    fprintf(graph, '%d\n', Temp);              %write temperature values to text file
   
  
    %t=linspace(0,2.833*10^4,200);
    %fprintf(graph, '%d\n', t);
    %disp(Temp)
    % plot the data on the screen
    %if d(1)>d(15)
    %A=load('test1.txt'); 
    %plot(t,A);
    plot(Temp);                          %plot current temperature 
    %end
    % Here are examples on how to set graph title, labels and axes
    title('EZ Scope');
    xlabel('Time - sec')
    ylabel('temperature (C)')
    axis ([0 NPOINTS 0 60])
  

    %plot(t,A);
    % use drawnow to update the figure
    drawnow  
    %figure
    s=serial('COM7');                     
    clear s                             %clear serial port 
  end

  
  
else
  RUN = 0;
  % change the string on the button to RUN
  set(handles.Run_Button,'string','RUN')
end

%----------------------------------------------------------------------

 T
 P
 I
 D
% use the below command in MATLAB command line to create temperature plot over time: 
 %data=load('test1.txt');plot(data);title('T=50, P=1, I=1, D=1');xlabel('points');ylabel('temperature (C)')          
