function map = cam2map( x1,y1,x2,y2 )
%UNTITLED Summary of this function goes here
%   Function for obtaining X,Y,Z from frame.
ImgCam1 = [ x1, y1, 1]';
ImgCam2 = [ x2, y2, 1]';
  b = 0.111; % baseline
  
  p = 5.5e-6;% pixel size

  f_n = 0.030/p; % focal length in pixels

  Cpx_n = 1024; %principal point in pixels

  Cpy_n = 1024;

  deltaCpx = 0; % error in principal point in pixels

  deltaCpy = 0;

  fError = 0*f_n; % focal length error

  siftError = 0;

  pitch_n = 0*pi/180;

  yaw_n = 0*pi/180;

 

  %pitch  - tilt - rotation around X axis
  %yaw  - pan - rotation around Y axis

  pitch_E = 0*pi/180;%pitch error
  yaw_E = 0*pi/180;%yaw error

  K=[f_n 0 Cpx_n;0 f_n Cpy_n; 0 0 1]; %Intrinsic Matrix
   % Converting to optical axis frame

    R1_n = [1 0 0; 0 cos(pitch_n) sin(pitch_n); 0 -sin(pitch_n) cos(pitch_n)];
    R3_n = [cos(yaw_n) sin(yaw_n) 0; -sin(yaw_n) cos(yaw_n) 0; 0 0 1];
    
  Kinv_n = inv(K); %K inverse
  deltaKinv = [(-fError/f_n^2) 0 ((deltaCpx/f_n)-Cpx_n*fError/f_n^2); 0 (-fError/f_n^2) ((-deltaCpy/f_n)+Cpy_n*fError/f_n^2); 0 0 0];

  deltaImgCam1 = [siftError siftError 0]';
  deltaImgCam2 = [siftError siftError 0]';
  d_n = ImgCam1(1)-ImgCam2(1); %Disparity
  deltad = 2*siftError; %Disparity Error

  Z_n = f_n*b/(d_n); %Depth of point
  deltaZ = b*fError/d_n-b*f_n*deltad/d_n^2; %Depth Error
  PC11_n = Z_n*Kinv_n*ImgCam1; %World coordinate in Camera1 frame without error.
  PC22_n = Z_n*Kinv_n*ImgCam2;
  deltaPC11 = deltaZ*Kinv_n*ImgCam1 + Z_n*deltaKinv*ImgCam1 + Z_n*Kinv_n*deltaImgCam1;
  PC11_net = PC11_n+ deltaPC11;
  POnet = PC11_n - [b/2 0 0]'; % Translating to rotated O Frame
  WPC11_n = R1_n'*R3_n'*POnet; % Coordinates in non-rotated O Frame
  
  
  %Error in rotation Matrix
  deltaR1 = [0 0 0; 0 -sin(pitch_n)*pitch_E cos(pitch_n)*pitch_E; 0 -cos(pitch_n)*pitch_E -sin(pitch_n)*pitch_E];
  R1_E = R1_n+deltaR1;
  deltaR3 = [-sin(yaw_n)*yaw_E cos(yaw_n)*yaw_E 0; -cos(yaw_n)*yaw_E -sin(yaw_n)*yaw_E 0; 0 0 0];
  R3_E = R3_n+ deltaR3;
  
  
  % Error in world coordinates in non-rotated O Frame
  deltaWPC11 = deltaR1'*R3_n'*POnet + R1_n'*deltaR3'*POnet + R1_n'*R3_n'*deltaPC11;
  WPC11_E = WPC11_n + deltaWPC11;
  map = WPC11_E;
end