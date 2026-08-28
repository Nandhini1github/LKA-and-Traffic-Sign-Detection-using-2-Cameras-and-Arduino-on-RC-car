%% Lane Detection + Stanley Controller
%  Pipeline: Capture -> Undistort -> Preprocess -> Hough -> Classify -> Fit ->
%            Track -> CTE -> Stanley -> Arduino PWM
%
clear; clc; close all;

%% ── PARAMETERS ──────────────────────────────────────────────────────────

% Throttle
THROTTLE_NEUTRAL = 0.50;
MOVING_THROTTLE  = 0.597;  

% Steering
STEERING_CENTER  = 0.423;   % PWM fraction for straight wheels
MAX_STEER_DELTA  = 0.33;    % max deviation from centre
STEER_SIGN       = +1;      % +1 or -1 depending on servo wiring

% Stanley controller gains
K_STANLEY  = 1.10;   
K_SOFT     = 0.05;   
V_REF      = 0.610;  % nominal speed — keep in sync with MOVING_THROTTLE
ALPHA_LPF  = 0.75;  

% Curve feedforward pre-steer
FEEDFORWARD_DELTA = 0.18;   % initial bump in radians when curve starts
FEEDFORWARD_DECAY = 0.75;   % how fast the bump fades each frame

% Cross-track error settings
CTE_DEADBAND    = 0.05;   % ignores noise near centre
REF_Y_NEAR_FRAC = 0.80;   % near look-ahead row (fraction of image height)
REF_Y_FAR_FRAC  = 0.55;   % far  look-ahead row
CTE_FAR_WEIGHT  = 0.40;   % blend weight for far horizon CTE

% CTE spike rejection — discard obviously bad detections
CTE_MAX_VALID   = 0.70;   % |CTE| above this treated as bad frame

% Lane tracking
DUAL_CONFIRM  = 1;    % frames needed to confirm both lanes back
TRACK_MAX_AGE = 12;   % holds lost lane longer
TRACK_DECAY   = 0.08; % how fast a stale track drifts toward centre

% Camera / ROI
CAM_INDEX  = 2;
CAP_W = 640; CAP_H = 480;   % capture resolution
PROC_W= 320; PROC_H= 240;   % processing resolution (smaller = faster)
ROI_TOP = 0.35;  % top edge of lane region (cuts out ceiling)
ROI_BOT = 0.93;  % bottom edge (cuts out car bonnet)

% Image processing
BLUR_SIGMA    = 1.0;    % Gaussian blur strength
BINARIZE_SENS = 0.40;   % adaptive threshold sensitivity

% Hough transform
H_RHO_RES   = 1;
H_THETA_RES = 1.0;
H_THRESH    = 20;    
H_FILL_GAP  = 20;   
H_MIN_LEN   = 15;    
H_MAX_PEAKS = 16;    % max lines returned from Hough

% Segment filter
MIN_SLOPE      = 0.20;   % reject near-horizontal lines
MAX_SLOPE      = 8.0;    % reject near-vertical lines
EXCLUSION_BAND = 15;     % ignore segments too close to image centre (px)
HALF_LANE_FRAC = 0.42;   % estimated half-lane width as fraction of PROC_W

% Arduino
ARDUINO_PORT  = 'COM3';
ARDUINO_BOARD = 'Uno';
STEER_PIN = 'D9';  ESC_PIN = 'D13';
PWM_MIN_US = 1000; PWM_MAX_US = 2000;

%% ── CAMERA INTRINSICS (from calibration at 1280x720, scaled to 640x480) ─
%
%  Original calibration resolution : 1280 x 720
%  Capture resolution               :  640 x 480  -> scale factor = 0.5
%
%  Pixel-based parameters (fx, fy, cx, cy) scale linearly with resolution.
%  Distortion coefficients (k1,k2,k3,p1,p2) are dimensionless — no scaling.
%
%  Extracted from lane_camera_params.mat:
%    fx_calib = 1038.216  fy_calib = 1038.410
%    cx_calib =  648.296  cy_calib =  336.056
%    RadialDistortion     = [-0.1168, -0.7849,  4.5733]
%    TangentialDistortion = [-0.00984, -0.00522]

SCALE_CAP = CAP_W / 1280;   % = 0.5

fx_cap = 1038.216 * SCALE_CAP;   % 519.108
fy_cap = 1038.410 * SCALE_CAP;   % 519.205
cx_cap =  648.296 * SCALE_CAP;   % 324.148
cy_cap =  336.056 * SCALE_CAP;   % 168.028

% Build cameraIntrinsics object at capture resolution (640x480)
cam_intrinsics = cameraIntrinsics( ...
    [fx_cap, fy_cap], ...
    [cx_cap, cy_cap], ...
    [CAP_H,  CAP_W], ...
    'RadialDistortion',     [-0.1168, -0.7849,  4.5733], ...
    'TangentialDistortion', [-0.00984, -0.00522]);

%% ── STARTUP CHECKS ───────────────────────────────────────────────────────

if MOVING_THROTTLE < THROTTLE_NEUTRAL
    error('MOVING_THROTTLE must be greater than THROTTLE_NEUTRAL');
end
if ~ismember(STEER_SIGN, [-1, 1])
    error('STEER_SIGN must be +1 or -1');
end

%% ── HARDWARE INIT ────────────────────────────────────────────────────────

% Open camera
cam = webcam(CAM_INDEX);
cam.Resolution = sprintf('%dx%d', CAP_W, CAP_H);
snapshot(cam);

% Open Arduino and create servo objects
ard = arduino(ARDUINO_PORT, ARDUINO_BOARD, 'Libraries', 'Servo');
steer_servo = servo(ard, STEER_PIN, ...
    'MinPulseDuration', PWM_MIN_US*1e-6, 'MaxPulseDuration', PWM_MAX_US*1e-6);
esc_servo = servo(ard, ESC_PIN, ...
    'MinPulseDuration', PWM_MIN_US*1e-6, 'MaxPulseDuration', PWM_MAX_US*1e-6);

% Centre steering
writePosition(steer_servo, STEERING_CENTER);
pause(1.0);

% Arm the ESC (must see neutral for ~3 s before accepting drive commands)
writePosition(esc_servo, THROTTLE_NEUTRAL);
pause(3.0);
writePosition(esc_servo, THROTTLE_NEUTRAL + 0.04);
pause(0.25);
writePosition(esc_servo, THROTTLE_NEUTRAL);
pause(1.0);

%% ── DISPLAY WINDOW ───────────────────────────────────────────────────────

hFig = figure('Name','Lane Tracker v12','NumberTitle','off',...
              'Color','k','Position',[100 100 700 560]);
hAx  = axes(hFig);

%% ── PRE-COMPUTE CONSTANTS ────────────────────────────────────────────────

cx        = PROC_W / 2;
half_w    = PROC_W / 2;
half_lane = PROC_W * HALF_LANE_FRAC;
r_top     = round(ROI_TOP * PROC_H);
r_bot     = round(ROI_BOT * PROC_H);
ref_y_near= round(REF_Y_NEAR_FRAC * PROC_H);
ref_y_far = round(REF_Y_FAR_FRAC  * PROC_H);
theta_vec = -90 : H_THETA_RES : 89.5;

% ROI mask — only process pixels in the lane region
roi_mask = false(PROC_H, PROC_W);
roi_mask(r_top:r_bot, :) = true;

MAX_SEGS  = 60;
left_buf  = zeros(MAX_SEGS, 5);
right_buf = zeros(MAX_SEGS, 5);

%% ── PERSISTENT LANE TRACK STATE ─────────────────────────────────────────
% Each track holds the last known fitted line for left/right lane.
% 'age' counts frames since last fresh detection (0 = detected this frame).

ltrack = struct('x1',cx-half_lane,'y1',PROC_H, ...
                'x2',cx-half_lane,'y2',round(ROI_TOP*PROC_H), ...
                'valid',false,'age',TRACK_MAX_AGE+1);
rtrack = struct('x1',cx+half_lane,'y1',PROC_H, ...
                'x2',cx+half_lane,'y2',round(ROI_TOP*PROC_H), ...
                'valid',false,'age',TRACK_MAX_AGE+1);

%% ── SIGN COMMAND SERVER (WiFi TCP listener) ──────────────────────────────
% Open a non-blocking TCP server socket. Laptop 2 (sign PC) connects and
% sends one-line commands: "stop", "school_zone", "slow", or "clear".
% We poll this socket each control loop iteration with a 0-ms timeout so
% it never stalls the real-time loop.
%
% Valid commands and their effect on throttle:
%   "stop"        -> THROTTLE_NEUTRAL  (vehicle halts, steering centred)
%   "school_zone" -> HALF_THROTTLE     (half cruising speed)
%   "slow"        -> HALF_THROTTLE     (same as school_zone)
%   "clear"       -> MOVING_THROTTLE   (normal cruising)
%   "none"        -> MOVING_THROTTLE   (default / not yet received any cmd)

SIGN_SERVER_PORT = 5005;   % must match VEHICLE_PORT on Laptop 2

signServerSock = [];
signClientSock = [];
signReader     = [];
signCommand    = "none";   % current active sign command
% Traffic sign timing logic
STOP_HOLD_SEC     = 2.0;   % STOP for 2 seconds
SCHOOL_LOST_SEC   = 1.0;   % resume if SCHOOL not seen for 1 sec

trafficClock = tic;
activeMode = "clear";      % clear | stop | school
stopEndTime = -inf;
lastSchoolSeenTime = -inf;
try
    signServerSock = java.net.ServerSocket(SIGN_SERVER_PORT);
    signServerSock.setSoTimeout(1);   % 1 ms accept timeout (non-blocking)
    fprintf('[SIGN SERVER] Listening on port %d ...\n', SIGN_SERVER_PORT);
catch ex
    fprintf('[SIGN SERVER] Could not open server socket: %s\n', ex.message);
end

%% ───────────────────────────────────────────
% HALF_THROTTLE = School Zone slow Speed
HALF_THROTTLE = 0.5864;

fprintf('[SIGN] Throttle levels:\n');
fprintf('  THROTTLE_NEUTRAL = %.4f  (stopped)\n',  THROTTLE_NEUTRAL);
fprintf('  HALF_THROTTLE    = %.4f  (school zone / slow)\n', HALF_THROTTLE);
fprintf('  MOVING_THROTTLE  = %.4f  (normal cruising)\n',    MOVING_THROTTLE);

%% ── MAIN CONTROL LOOP ────────────────────────────────────────────────────

loop_n        = 0;
t_start       = tic;
delta_prev    = 0;
ff_delta      = 0;
was_slope_mode= false;
cte_prev      = 0;   % for spike rejection fallback

try
    while ishandle(hFig)

        loop_n = loop_n + 1;

        % ── Poll sign command server ───────────────────────────────────────
        % Accept a new client connection if none is connected yet.
        % If the client disconnects, reset so a reconnect is accepted.
        if ~isempty(signServerSock)
            if isempty(signClientSock)
                try
                    signClientSock = signServerSock.accept();   % 1 ms timeout
                    signClientSock.setSoTimeout(1);             % non-blocking reads
                    inStream = signClientSock.getInputStream();
                    signReader = java.io.BufferedReader( ...
                        java.io.InputStreamReader(inStream));
                    fprintf('[SIGN SERVER] Sign PC connected.\n');
                catch
                    % No connection yet — normal, keep looping
                end
            end

            % Try to read a command line (non-blocking due to setSoTimeout).
            if ~isempty(signReader)
                try
                    line = signReader.readLine();
                    if ~isempty(line)
                        cmd = strtrim(string(line));
                        % Accept: stop | school_zone | slow | clear
                        % "slow" maps to the same behaviour as "school_zone"
                        cmd = lower(strtrim(string(line)));
                        if cmd == "ping"
    outStream = signClientSock.getOutputStream();
    writer = java.io.PrintWriter(outStream, true);
    writer.print(['pong' char(13) char(10)]);
    outStream.flush();
    fprintf('[SIGN] Ping received -> pong sent\n');
    continue;
     end
                        if cmd == "slow" || cmd == "school_zone" || cmd == "school"
                        cmd = "school";
                        end

                        if ismember(cmd, ["stop", "school", "clear"])

    signCommand = cmd;
    nowTime = toc(trafficClock);

    if cmd == "stop"
        activeMode = "stop";
        stopEndTime = nowTime + STOP_HOLD_SEC;

    elseif cmd == "school"
        activeMode = "school";
        lastSchoolSeenTime = nowTime;

    elseif cmd == "clear"
        activeMode = "clear";
    end

    fprintf('[SIGN] Command received: %s | activeMode=%s\n', cmd, activeMode);

else
    fprintf('[SIGN] Unknown command ignored: %s\n', cmd);
end
                    end
                catch socketEx
                    % readLine throws SocketException on disconnect
                    if contains(socketEx.message, 'reset', 'IgnoreCase', true) || ...
                       contains(socketEx.message, 'closed', 'IgnoreCase', true)
                        fprintf('[SIGN SERVER] Sign PC disconnected — will accept new connection.\n');
                        try, signClientSock.close(); catch, end
                        signClientSock = [];
                        signReader     = [];
                        signCommand    = "none";   % safe default when sign PC gone
                    end
                    % Timeout (normal non-blocking miss) falls through silently
                end
            end
        end
        % ── END sign server poll ──────────────────────────────────────────

        % ── Capture ───────────────────────────────────────────────────────
        frame_full = snapshot(cam);

        % ── UNDISTORT at capture resolution (640x480) ─────────────────────
        % 'OutputView','same' crops the undistorted image back to the
        % original frame size, eliminating the black border that appeared
        % in v11 when the barrel distortion was corrected.
        frame_full = undistortImage(frame_full, cam_intrinsics, ...
                                    'OutputView', 'same');

        % ── Resize for processing ─────────────────────────────────────────
        frame = imresize(frame_full, [PROC_H PROC_W]);

        % ── Pre-process: grayscale -> blur -> binarize -> edges ──────────
        gray     = rgb2gray(frame);
        blur_img = imgaussfilt(double(gray), BLUR_SIGMA);
        bin      = imbinarize(uint8(blur_img), 'adaptive', ...
                       'ForegroundPolarity','bright','Sensitivity',BINARIZE_SENS);
        roi_bin  = bin & roi_mask;
        edges    = edge(roi_bin, 'Canny');

        % ── Hough transform: find line segments in edge image ─────────────
        [H_acc, theta, rho] = hough(edges, ...
            'RhoResolution',H_RHO_RES,'Theta',theta_vec);
        peaks = houghpeaks(H_acc, H_MAX_PEAKS, ...
            'Threshold',H_THRESH,'NHoodSize',[5 5]);
        if isempty(peaks)
            lines = struct([]);
        else
            lines = houghlines(edges, theta, rho, peaks, ...
                'FillGap',H_FILL_GAP,'MinLength',H_MIN_LEN);
        end

        % ── Classify segments as LEFT or RIGHT using bottom-intercept ─────
        n_left = 0; n_right = 0;

        for k = 1:numel(lines)
            x1=lines(k).point1(1); y1=lines(k).point1(2);
            x2=lines(k).point2(1); y2=lines(k).point2(2);
            dx=x2-x1; dy=y2-y1;
            if abs(dx)<1, continue; end
            slope = dy/dx;
            if abs(slope)<MIN_SLOPE || abs(slope)>MAX_SLOPE, continue; end
            mid_x = (x1+x2)*0.5;
            if abs(mid_x-cx)<EXCLUSION_BAND, continue; end

            % Bottom intercept
            if abs(dy)>0.5
                x_bot = x1 + (x2-x1)*(PROC_H-y1)/(y2-y1);
            else
                x_bot = mid_x;
            end

            row = [x1 y1 x2 y2 slope];
            if x_bot < cx
                n_left=n_left+1;
                if n_left<=MAX_SEGS, left_buf(n_left,:)=row; end
            else
                n_right=n_right+1;
                if n_right<=MAX_SEGS, right_buf(n_right,:)=row; end
            end
        end

        L = left_buf(1:n_left,:);
        R = right_buf(1:n_right,:);

        % ── Fit one line per side (weighted least squares) ────────────────
        [lx1f,ly1f,lx2f,ly2f,lv_new] = fit_lane(L, PROC_H, ROI_TOP);
        [rx1f,ry1f,rx2f,ry2f,rv_new] = fit_lane(R, PROC_H, ROI_TOP);

        % ── Update persistent tracks (carry forward if lane lost) ─────────
        if lv_new
            ltrack.x1=lx1f; ltrack.y1=ly1f; ltrack.x2=lx2f; ltrack.y2=ly2f;
            ltrack.valid=true; ltrack.age=0;
        else
            ltrack.age = ltrack.age+1;
            if ltrack.valid && ltrack.age<=TRACK_MAX_AGE
                ltrack.x1=ltrack.x1+TRACK_DECAY*(cx-ltrack.x1);
                ltrack.x2=ltrack.x2+TRACK_DECAY*(cx-ltrack.x2);
                lv_new=true; lx1f=ltrack.x1; ly1f=ltrack.y1;
                              lx2f=ltrack.x2; ly2f=ltrack.y2;
            else
                ltrack.valid=false;
            end
        end

        if rv_new
            rtrack.x1=rx1f; rtrack.y1=ry1f; rtrack.x2=rx2f; rtrack.y2=ry2f;
            rtrack.valid=true; rtrack.age=0;
        else
            rtrack.age = rtrack.age+1;
            if rtrack.valid && rtrack.age<=TRACK_MAX_AGE
                rtrack.x1=rtrack.x1+TRACK_DECAY*(cx-rtrack.x1);
                rtrack.x2=rtrack.x2+TRACK_DECAY*(cx-rtrack.x2);
                rv_new=true; rx1f=rtrack.x1; ry1f=rtrack.y1;
                              rx2f=rtrack.x2; ry2f=rtrack.y2;
            else
                rtrack.valid=false;
            end
        end

        lv=lv_new; rv=rv_new;
        lx1=lx1f; ly1=ly1f; lx2=lx2f; ly2=ly2f;
        rx1=rx1f; ry1=ry1f; rx2=rx2f; ry2=ry2f;

        % ── Heading error: angle between car axis and lane direction ───────
        psi_e = heading_error_geometry(lx1,ly1,lx2,ly2,lv, ...
                                       rx1,ry1,rx2,ry2,rv);
        psi_e = max(-deg2rad(25), min(deg2rad(25), psi_e));

        % ── Cross-track error (CTE): how far car is from lane centre ──────
        both_near = (lv && rv);

        if both_near
            lcx_near = lane_centre_both(lx1,ly1,lx2,ly2,rx1,ry1,rx2,ry2,ref_y_near);
            lcx_far  = lane_centre_both(lx1,ly1,lx2,ly2,rx1,ry1,rx2,ry2,ref_y_far);
            cte_near = (lcx_near - cx) / half_w;
            cte_far  = (lcx_far  - cx) / half_w;
            cte = (1-CTE_FAR_WEIGHT)*cte_near + CTE_FAR_WEIGHT*cte_far;
            if abs(cte)<CTE_DEADBAND, cte=0; end
            detected=2; lcx_disp=lcx_near;

        elseif lv
            lc_x_raw = x_at_y(lx1,ly1,lx2,ly2,ref_y_near) + half_lane;
            lcx_near = max(half_lane, min(PROC_W-half_lane, lc_x_raw));
            cte=(lcx_near-cx)/half_w; detected=1; lcx_disp=lcx_near;

        elseif rv
            lc_x_raw = x_at_y(rx1,ry1,rx2,ry2,ref_y_near) - half_lane;
            lcx_near = max(half_lane, min(PROC_W-half_lane, lc_x_raw));
            cte=(lcx_near-cx)/half_w; detected=1; lcx_disp=lcx_near;

        else
            cte=0; detected=0; lcx_disp=cx; lcx_near=cx;
        end

        % ── CTE spike rejection ────────────────────────────────────────────
        % If CTE jumps to an implausible value (e.g. the -0.877 spike seen
        % in the telemetry), reuse the previous frame's CTE instead.
        if abs(cte) > CTE_MAX_VALID
            fprintf('[WARN] CTE spike rejected: %+.3f -> using prev %+.3f\n', ...
                    cte, cte_prev);
            cte = cte_prev;
        end
        cte_prev = cte;

        % ── Curve feedforward: pre-steer bump at curve entry ──────────────
        is_slope_mode_now = (detected==1);
        if is_slope_mode_now && ~was_slope_mode
            ff_sign = sign(cte); if ff_sign==0, ff_sign=-1; end
            ff_delta = ff_sign * FEEDFORWARD_DELTA;
        else
            ff_delta = ff_delta * FEEDFORWARD_DECAY;
        end
        was_slope_mode = is_slope_mode_now;

        % ── Stanley control law: delta = psi_e + atan(K*CTE / v) ─────────
        stanley_delta = psi_e + atan2(K_STANLEY*cte, V_REF+K_SOFT);
        delta         = stanley_delta + ff_delta;

        % Low-pass filter to smooth servo output
        delta_smooth = ALPHA_LPF*delta + (1-ALPHA_LPF)*delta_prev;
        delta_prev   = delta_smooth;

        % Normalise to servo range and clamp
        delta_norm = max(-1.0, min(1.0, delta_smooth/deg2rad(40)));
        steer_out  = STEERING_CENTER + STEER_SIGN*delta_norm*MAX_STEER_DELTA;
        steer_out  = max(STEERING_CENTER-MAX_STEER_DELTA, ...
                     min(STEERING_CENTER+MAX_STEER_DELTA, steer_out));

        throttle_out = max(THROTTLE_NEUTRAL, MOVING_THROTTLE);

        % ── Apply sign-based throttle (and steer) override ────────────────
        %
        % signCommand is updated each loop by the TCP poll block above.
        % "slow" is normalised to "school_zone" on receipt, so only three
        % states need to be handled here:
        %
        %   "stop"        -> throttle = THROTTLE_NEUTRAL  (halt)
        %                    steer    = STEERING_CENTER   (centre wheels while stopped)
        %   "school_zone" -> throttle = HALF_THROTTLE    
        %                    steer unchanged (lane controller still active)
        %   "clear"/"none"-> throttle = MOVING_THROTTLE   (normal cruising)
        %                    steer unchanged
        % Auto timing update
nowTime = toc(trafficClock);

% STOP: stop for 2 seconds, then resume
if activeMode == "stop" && nowTime >= stopEndTime
    activeMode = "clear";
end

% SCHOOL: stay slow while school command keeps coming;
% resume if school command not received for 1 second
if activeMode == "school" && (nowTime - lastSchoolSeenTime > SCHOOL_LOST_SEC)
    activeMode = "clear";
end

% Apply traffic mode
if activeMode == "stop"
    throttle_out = THROTTLE_NEUTRAL;
    steer_out    = STEERING_CENTER;

elseif activeMode == "school"
    throttle_out = HALF_THROTTLE;

else
    throttle_out = MOVING_THROTTLE;
end
        % "clear" and "none" fall through with throttle_out = MOVING_THROTTLE
        % ── END sign override ─────────────────────────────────────────────

        % ── Write to Arduino ──────────────────────────────────────────────
        writePosition(steer_servo, steer_out);
        writePosition(esc_servo,   throttle_out);

        % ── Telemetry (every 15 frames) ───────────────────────────────────
        if mod(loop_n,15)==0
            fps = loop_n/toc(t_start);
            fprintf('[%05d] FPS=%4.1f det=%d Lage=%d Rage=%d | CTE=%+.3f | Steer=%.4f | Thr=%.4f | Sign=%s\n',...
                loop_n,fps,detected,ltrack.age,rtrack.age,cte,steer_out,throttle_out,signCommand);
        end

        % ── Live display ──────────────────────────────────────────────────
        imshow(frame_full,'Parent',hAx); hold(hAx,'on');
        sx=CAP_W/PROC_W; sy=CAP_H/PROC_H;

        % Raw Hough lines (blue)
        for k=1:numel(lines)
            plot(hAx,[lines(k).point1(1) lines(k).point2(1)]*sx,...
                     [lines(k).point1(2) lines(k).point2(2)]*sy,'b-','LineWidth',1);
        end

        % Fitted lanes: solid=fresh, dashed=from track
        lstyle='g-'; if lv&&ltrack.age>0, lstyle='g--'; end
        rstyle='r-'; if rv&&rtrack.age>0, rstyle='r--'; end
        if lv, plot(hAx,[lx1 lx2]*sx,[ly1 ly2]*sy,lstyle,'LineWidth',3); end
        if rv, plot(hAx,[rx1 rx2]*sx,[ry1 ry2]*sy,rstyle,'LineWidth',3); end

        % Lane centre and CTE
        plot(hAx,lcx_disp*sx,ref_y_near*sy,'yo','MarkerSize',12,'LineWidth',2.5);
        if both_near
            lcx_far_d=lane_centre_both(lx1,ly1,lx2,ly2,rx1,ry1,rx2,ry2,ref_y_far);
            plot(hAx,lcx_far_d*sx,ref_y_far*sy,'mo','MarkerSize',12,'LineWidth',2.5);
        end
        plot(hAx,cx*sx,ref_y_near*sy,'w+','MarkerSize',12,'LineWidth',2.5);
        plot(hAx,[cx*sx lcx_disp*sx],[ref_y_near*sy ref_y_near*sy],'y--','LineWidth',2);

        % ROI box and exclusion band
        plot(hAx,[1 CAP_W CAP_W 1 1],[r_top r_top r_bot r_bot r_top]*sy,'c:','LineWidth',1);
        plot(hAx,[(cx-EXCLUSION_BAND)*sx (cx-EXCLUSION_BAND)*sx],[r_top*sy r_bot*sy],'m:','LineWidth',1);
        plot(hAx,[(cx+EXCLUSION_BAND)*sx (cx+EXCLUSION_BAND)*sx],[r_top*sy r_bot*sy],'m:','LineWidth',1);

        % Steering arrow (cyan when feedforward active)
        arrow_x=cx*sx+STEER_SIGN*delta_norm*120;
        arr_col='m'; if abs(ff_delta)>0.01, arr_col='c'; end
        plot(hAx,[cx*sx arrow_x],[CAP_H-20 CAP_H-20],[arr_col '-'],'LineWidth',6);
        plot(hAx,cx*sx,CAP_H-20,'w.','MarkerSize',14);

        if detected==2, dcol='g'; dstr='BOTH';
        elseif detected==1, dcol='y'; dstr='ONE';
        else, dcol='r'; dstr='NONE'; end

        title(hAx,sprintf('[%s] CTE=%+.3f  ff=%+.3f  psi=%+.1fd  Steer=%.4f  L-age=%d R-age=%d  Sign=%s',...
            dstr,cte,ff_delta,rad2deg(psi_e),steer_out,ltrack.age,rtrack.age,signCommand),...
            'Color',dcol,'FontSize',9);

        hold(hAx,'off');
        drawnow limitrate;

    end % while

catch ME
    fprintf('\n[ERROR] %s -> %s line %d\n',ME.message,ME.stack(1).name,ME.stack(1).line);
end

%% ── SHUTDOWN ─────────────────────────────────────────────────────────────

fprintf('\n[SHUTDOWN] Stopping ...\n');
try
    writePosition(esc_servo,   THROTTLE_NEUTRAL);
    writePosition(steer_servo, STEERING_CENTER);
    pause(0.5); clear ard;
catch
    fprintf('[SHUTDOWN] Arduino may already be disconnected.\n');
end
clear cam;

% Close sign command server sockets
try
    if ~isempty(signClientSock), signClientSock.close(); end
    if ~isempty(signServerSock), signServerSock.close(); end
catch
    fprintf('[SHUTDOWN] Sign server socket already closed.\n');
end

fprintf('[SHUTDOWN] Done.\n');


%% ── LOCAL FUNCTIONS ──────────────────────────────────────────────────────

% Weighted least-squares fit of all segments on one side into one line.
% Returns bottom and top endpoints spanning the ROI, plus a valid flag.
function [x1o,y1o,x2o,y2o,valid] = fit_lane(segs, img_h, roi_top_frac)
    valid=false; x1o=0; y1o=0; x2o=0; y2o=0;
    if isempty(segs), return; end
    xs=[segs(:,1);segs(:,3)]; ys=[segs(:,2);segs(:,4)];
    if numel(xs)<2, return; end
    seg_len=max(hypot(segs(:,3)-segs(:,1),segs(:,4)-segs(:,2)),1);
    w=[seg_len;seg_len];
    A=[ys ones(numel(ys),1)]; W=diag(w);
    coef=(A'*W*A)\(A'*W*xs);
    y1o=img_h; y2o=round(roi_top_frac*img_h);
    x1o=round(coef(1)*y1o+coef(2)); x2o=round(coef(1)*y2o+coef(2));
    valid=true;
end

% Returns x coordinate of a line at a given row y.
function xi = x_at_y(x1,y1,x2,y2,yq)
    if abs(y2-y1)<1, xi=(x1+x2)*0.5;
    else, xi=x1+(x2-x1)*(yq-y1)/(y2-y1); end
end

% Returns the midpoint x between left and right lanes at a given row.
function lc_x = lane_centre_both(lx1,ly1,lx2,ly2,rx1,ry1,rx2,ry2,yrow)
    lc=x_at_y(lx1,ly1,lx2,ly2,yrow);
    rc=x_at_y(rx1,ry1,rx2,ry2,yrow);
    lc_x=(lc+rc)*0.5;
end

% Returns mean heading angle of detected lane lines relative to car axis.
function psi = heading_error_geometry(lx1,ly1,lx2,ly2,lv,rx1,ry1,rx2,ry2,rv)
    angles=[];
    if lv, dx_l=lx2-lx1; dy_l=ly2-ly1;
        if abs(dy_l)>1, angles=[angles,atan2(dx_l,-dy_l)]; end; end
    if rv, dx_r=rx2-rx1; dy_r=ry2-ry1;
        if abs(dy_r)>1, angles=[angles,atan2(dx_r,-dy_r)]; end; end
    if isempty(angles), psi=0; else, psi=mean(angles); end
end
