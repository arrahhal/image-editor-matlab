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

function b = initImg()
    global imgs idx img
    img = imgs{idx};
    if isempty(img)
        msgbox('No image loaded!', 'Error', 'error');
        b = false;
    else
        b = true;
    end
end


function yuvimg = rgb2yuv(img)
    R = double(img(:,:,1)) / 255;
    G = double(img(:,:,2)) / 255;
    B = double(img(:,:,3)) / 255;
    
    Y = 0.299 * R + 0.587 * G + 0.114 * B;
    U = -0.14713 * R - 0.28886 * G + 0.436 * B;
    V = 0.615 * R - 0.51499 * G - 0.10001 * B;
    YUV = cat(3,Y,U,V);
    yuvimg = YUV;
end

function cmykimg = rgb2cmyk(img)
    R = double(img(:,:,1)) / 255;
    G = double(img(:,:,2)) / 255;
    B = double(img(:,:,3)) / 255;
    
    C_prime = 1 - R;
    M_prime = 1 - G;
    Y_prime = 1 - B;
    
    K = min(min(C_prime, M_prime), Y_prime);
    
    denom = 1 - K;
    denom(denom == 0) = 1;
    
    C = (C_prime - K) ./ denom;
    M = (M_prime - K) ./ denom;
    Y = (Y_prime - K) ./ denom;
    
    cmykimg = cat(3, C, M, Y, K);
end

function rgbimg = cmyk2rgb(cmykimg)
    % Extract CMYK channels
    C = cmykimg(:,:,1);
    M = cmykimg(:,:,2);
    Y = cmykimg(:,:,3);
    K = cmykimg(:,:,4);

    % Convert CMYK to RGB
    R = (1 - C) .* (1 - K);
    G = (1 - M) .* (1 - K);
    B = (1 - Y) .* (1 - K);

    % Combine the channels into an RGB image
    rgbimg = cat(3, R, G, B);

    % Scale back to [0, 255] range if needed
    rgbimg = uint8(rgbimg * 255);
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


function pushbutton4_Callback(hObject, eventdata, handles)
    global img
    if (~initImg())
        return;
    end
    img = imrotate(img, -90);
    commitImg();
end

function pushbutton12_Callback(hObject, eventdata, handles)
    global img
    if (~initImg())
        return;
    end
    img = imrotate(img, 90);
    commitImg();
end

function pushbutton6_Callback(hObject, eventdata, handles)
    global width height img
    if (~initImg())
        return;
    end
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

function pushbutton7_Callback(hObject, eventdata, handles)
    global img
    if (~initImg())
        return;
    end
    img = flip(img, 2);
    commitImg();
end

function popupmenu2_Callback(hObject, eventdata, handles)
    global img
    if (~initImg())
        return;
    end
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

function popupmenu2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function pushbutton9_Callback(hObject, eventdata, handles)
    global img
    if (~initImg())
        return;
    end
    if isempty(img)
        msgbox('No image loaded!', 'Error', 'error');
        return;
    end
    figure, imshow(img);
    title('???  ??????? ???? ???? ???? ?? ??????');
    croppedImg = imcrop;
    if isempty(croppedImg)
        msgbox('Cropping cancelled.', 'Info');
        return;
    end
 
    img = croppedImg;
    close;
    commitImg();
end

function slider2_Callback(hObject, eventdata, handles)
    global img
    if (~initImg())
        return;
    end
    brightnessFactor = get(hObject, 'Value');
    img = imadjust(img, [], [], brightnessFactor);
    commitImg();
end

function slider2_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
end

function popupmenu3_Callback(hObject, eventdata, handles)
    global img
    if (~initImg())
        return;
    end
    colorSystems = get(hObject, 'String');
    selectedColorSystem = colorSystems{get(hObject, 'Value')};
    switch selectedColorSystem
        case 'RGB'
        case 'Ycbcr'
            img = rgb2ycbcr(img);
        case 'YUV'
            img = rgb2yuv(img);
        case 'CMYK'
            cmykimg = rgb2cmyk(img);
            img = cmyk2rgb(cmykimg);
        case 'HSV'
            img = rgb2hsv(img);
        otherwise
            msgbox('Invalid color system selected!', 'Error', 'error');
            return;
    end
    commitImg();
end

function popupmenu3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

function pushbutton11_Callback(hObject, eventdata, handles)
    global imgs idx img
    img = imgs{idx};
    img = imresize(img, 0.5);
    commitImg();
end

function pushbutton10_Callback(hObject, eventdata, handles)
    imsave();
end

function popupmenu4_Callback(hObject, eventdata, handles)
    global img
    if(~initImg())
        return;
    end
    contents = cellstr(get(hObject, 'String'));
    value = contents{get(hObject, 'Value')};
    switch(value)
        case 'winter'
            m = colormap(winter);
            a = rgb2ind(img, m);
            img = ind2rgb(a,m);
        case 'hot'
            img(:,:,2:3) = 0;
        case 'summer'
            m = colormap(summer);
            a = rgb2ind(img, m);
            img = ind2rgb(a,m);
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


% --- Executes on slider movement.
function slider7_Callback(hObject, eventdata, handles)
    global img
    if (~initImg())
        return
    end
    lvl = get(hObject,'Value');
    img = imsharpen(img, 'Amount', lvl);
    commitImg();
end

% --- Executes during object creation, after setting all properties.
function slider7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
end


% --- Executes on slider movement.
function slider8_Callback(hObject, eventdata, handles)
    global img
    if (~initImg())
        return;
    end
    contrastValue = get(hObject, 'Value');
    img = imadjust(img, stretchlim(img), [], contrastValue);
    commitImg();
end

% --- Executes during object creation, after setting all properties.
function slider8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
end
