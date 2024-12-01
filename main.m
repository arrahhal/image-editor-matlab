function varargout = main(varargin)

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @main_OpeningFcn, ...
                   'gui_OutputFcn',  @main_OutputFcn, ...
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

end
% --- Executes just before main is made visible.
function main_OpeningFcn(hObject, eventdata, handles, varargin)
% Choose default command line output for main
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% my stuff 
global img imgs idx mx width height,
mx = 0;
img = [];
imgs = cell(1,10);
idx = 10;
end
% UIWAIT makes main wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = main_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;
end

function updateIdx()
    global idx imgs mx
    idx = min(idx, 10);
    idx = max(idx, 1);
    idx = max(idx, 10 - mx);
    imshow(imgs{idx});
end

% my stuff
function commitImg()
    global img imgs idx mx
    mx = mx + 1;
    if (idx ~= 10)
        img = imgs{idx};
        idx = 10;
    end
    imgs(1:9) = imgs(2:10);
    imgs{10} = img;
    imshow(img);
end

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
    global img idx
    [impath, nofile] = imgetfile();
    if (nofile)
        return
    end
    idx = 10;
    img = imread(impath);
    commitImg();
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
    global idx
    idx = idx - 1;
    updateIdx();
end

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
    global idx
    idx = idx + 1;
    updateIdx();
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
    global img
    img = imrotate(img, -90);
    commitImg();
end

function pushbutton6_Callback(hObject, eventdata, handles)
    global width height img
    img = imresize(img, [width, height]);
    commitImg();
end

function edit2_Callback(hObject, eventdata, handles)
    global height
    h = str2double(get(hObject, 'String'));
    height = h;

end
function edit2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function edit3_Callback(hObject, eventdata, handles)
    global width
    w = str2double(get(hObject, 'String'));
    width = w;
end

function edit3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
    global img
    img = flip(img, 2); % Mirror horizontally
    commitImg();
end

% --- Executes on button press in pushbutton8.
function popupmenu2_Callback(hObject, eventdata, handles)
    global img
    contents = cellstr(get(hObject, 'String'));
    selected = contents{get(hObject, 'Value')};
    
    switch selected
        case 'Laplacian'
            img = imfilter(img, fspecial('laplacian'));
        case 'Gaussian'
            img = imgaussfilt(img, 2); % Gaussian smoothing filter
        case 'Difference of Gaussian'
            img1 = imgaussfilt(img, 2);
            img2 = imgaussfilt(img, 4);
            img = img1 - img2;
        case 'Sobel'
            img = edge(rgb2gray(img), 'sobel');
        case 'Roberts'
            img = edge(rgb2gray(img), 'roberts');
    end
    commitImg();

end

% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
    global img
    if isempty(img)
        msgbox('No image loaded!', 'Error', 'error');
        return;
    end
    figure, imshow(img); % Open the image in a new figure
    title('Select a region to crop and double-click to confirm.');
    croppedImg = imcrop; % Let the user select a region

    % Check if the user completed the crop
    if isempty(croppedImg)
        msgbox('Cropping cancelled.', 'Info');
        return;
    end
    
    img = croppedImg; % Update the global image variable
    close; % Close the cropping figure
    commitImg(); % Commit and display the cropped image
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
    global img imgs idx
    % Get the slider value
    brightnessFactor = get(hObject, 'Value');
    img = imadjust(imgs{idx}, [], [], brightnessFactor);
    commitImg();
end

function slider2_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
end


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
    global img
    colorSystems = get(hObject, 'String');
    selectedColorSystem = colorSystems{get(hObject, 'Value')};
    switch selectedColorSystem
        case 'RGB'
            img = img; % Original RGB image
        case 'Ycbcr'
            img = rgb2ycbcr(img);
        case 'YUV'
            img = rgb2yuv(img);
        case 'CMYK'
            img = rgb2cmyk(img)
        case 'HSV'
            img = rgb2hsv(img);
        otherwise
            msgbox('Invalid color system selected!', 'Error', 'error');
            return;
    end
    commitImg();
end
% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function slider3_Callback(hObject, eventdata, handles)
    global img imgs idx
    if isempty(img)
        msgbox('No image loaded!', 'Error', 'error');
        return;
    end

    % Get the slider value
    sf = get(hObject, 'Value'); % Scaling factor

    % Resize the image using the scaling factor
    resizedImg = imresize(imgs{idx}, sf);

    img = resizedImg;
    commitImg();
end

function slider3_CreateFcn(hObject, eventdata, handles)
    set(hObject, 'Min', 0.1, 'Max', 2, 'Value', 1); % 10% to 200% scaling
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
end

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
    global imgs idx
    wcompress('c', imgs{idx}, 'temp.wtc', 'wdr');
    figure, imshow('temp.wtc');
    delete('temp.wtc');
end

function popupmenu4_Callback(hObject, eventdata, handles)
    global imgs idx img
    img = imgs{idx};
    contents = cellstr(get(hObject, 'String'));
    value = contents{get(hObject, 'Value')};
    switch(value)
        case 'gray'
            img(:, :, 2) = img(:, :, 1);
            img(:, :, 3) = img(:, :, 1);
        case 'hot'
            img(:,:,2:3) = 0;
        case 'cold'
            img(:, :,1:2) = 0;
        case 'greenish'
            img(:, :, 1) = 0;
            img(:, :, 3) = 0;
        case 'autumn'
            m = colormap(autumn);
            a = rgb2ind(img, m);
            img = ind2rgb(a,m);
    end
    commitImg();
end
function popupmenu4_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end
