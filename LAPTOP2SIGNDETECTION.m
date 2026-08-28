%% Sign Detection
clear; clc; close all;

fprintf('============================================================\n');
fprintf(' Sign Detection \n');
fprintf(' MATLAB %s   |   %s\n', version, datestr(now));
fprintf(' Tuned for Clemson indoor lab track (video analysis)\n');
fprintf('============================================================\n\n');

%% ── CONFIGURATION ────────────────────────────────────────────────────────

VEHICLE_IP   = '172.27.37.191';
VEHICLE_PORT = 5005;

CAM_W = 640;
CAM_H = 480;

% Camera orientation. rot180 = camera mounted upside down.
% Change to 'none', 'flipH', 'rot90cw', 'rot90ccw' if needed.
FLIP_MODE = 'rot180';

DETECTOR_FILE = 'roboCarSignDetector.mat';

% Detector confidence threshold
SCORE_THRESH = 0.15;

STOP_SUBSTRINGS        = ["stop"];
SCHOOL_ZONE_SUBSTRINGS = ["school","slow","zone","pedestrian","crosswalk"];

% ── Colour fallback —
USE_COLOR_FALLBACK = true;

% STOP sign (MATLAB rgb2hsv units: H 0-1, S 0-1, V 0-1)
% High saturation required to exclude dull background reds
RED_HUE_LOW1   = 0.00;   RED_HUE_HIGH1  = 0.055;  % hue 0-20° (primary)
RED_HUE_LOW2   = 0.945;  RED_HUE_HIGH2  = 1.00;   % hue 340-360° (wrap)
RED_SAT_MIN    = 0.58;                              % high sat = true red
RED_VAL_MIN    = 0.30;                              % allow dark areas
RED_BLOB_MIN   = 300;    % min pixels — sign is small, allow close approach
RED_BLOB_MAX   = 30000;  % max pixels — avoid giant background blobs
RED_ASPECT_MIN = 0.25;   % width/height min (not a horizontal wire)
RED_ASPECT_MAX = 2.8;    % width/height max (not a tall thin pipe)

% School Zone sign
YLW_HUE_LOW    = 0.08;   YLW_HUE_HIGH   = 0.185; 
YLW_SAT_MIN    = 0.55;                              
YLW_VAL_MIN    = 0.35;
YLW_BLOB_MIN   = 300;
YLW_BLOB_MAX   = 25000;
YLW_ASPECT_MIN = 0.25;
YLW_ASPECT_MAX = 2.5;

% Region-of-interest: only look for signs in upper 70% of frame
% (signs are above floor; lower 30% is just floor/lane markings)
ROI_TOP_FRAC    = 0.20;   % from this fraction of frame height
ROI_BOTTOM_FRAC = 0.68;  % to this fraction of frame height
% Only consider blobs whose centre is in centre 80% of frame width
% (eliminates toolboxes/equipment at left/right edges)
ROI_LEFT_FRAC   = 0.22;
ROI_RIGHT_FRAC  = 0.78;

ENHANCE_CONTRAST    = true;
CLEAR_FRAMES_NEEDED = 8;   
SHOW_VIDEO          = true;
CAM_MAX_RETRIES     = 3;
VERBOSE_FRAMES      = 20;

%% ── LOAD DETECTOR ────────────────────────────────────────────────────────

fprintf('[INIT] Loading detector: %s\n', DETECTOR_FILE);
if ~isfile(DETECTOR_FILE)
    error('Detector file not found: %s\n(pwd: %s)', DETECTOR_FILE, pwd);
end

warnState = warning('off','all');
tmp = load(DETECTOR_FILE);
warning(warnState);

fields   = fieldnames(tmp);
detector = tmp.(fields{1});
fprintf('[INIT] Loaded "%s"  class: %s\n', fields{1}, class(detector));

% Probe R2026a vs older API
DETECTOR_API = probeDetectorAPI(detector);
fprintf('[INIT] Detector API: %s\n', DETECTOR_API);

fprintf('[INIT] ClassNames: ');
try
    cn = detector.ClassNames;
    fprintf('%s\n\n', strjoin(string(cn),' | '));
catch
    fprintf('(unreadable)\n\n');
end

%% ── OPEN USB CAMERA ──────────────────────────────────────────────────────

fprintf('[CAM] Opening webcam("USB Camera") ...\n');
try
    cam = webcam("USB Camera");
catch ME
    fprintf('\n[CAM] ERROR: Cannot open "USB Camera".\n');
    fprintf('       Run webcamlist() to confirm name.\n');
    rethrow(ME);
end

try
    cam.Resolution = sprintf('%dx%d', CAM_W, CAM_H);
    fprintf('[CAM] Resolution: %s\n', cam.Resolution);
catch
    fprintf('[CAM] Using default resolution: %s\n', cam.Resolution);
end

%% ── ORIENTATION CHECK ────────────────────────────────────────────────────

rawFrame   = snapshot(cam);
corrFrame  = applyFlip(rawFrame, FLIP_MODE);
[fH,fW,~]  = size(corrFrame);

imwrite(corrFrame, 'diag_frame_v10.png');
fprintf('[INIT] diag_frame_v10.png saved.\n');

figure('Name','Orientation Check — close to continue','NumberTitle','off');
imshow(corrFrame);
title(sprintf('FLIP\\_MODE = "%s" | Does this look correct? Close to start.', FLIP_MODE), ...
      'Interpreter','none');
fprintf('[INIT] Close the orientation window to begin detection.\n\n');
waitfor(gcf);

%% ── TCP CONNECTION ───────────────────────────────────────────────────────

fprintf('[INIT] TCP → %s:%d ...\n', VEHICLE_IP, VEHICLE_PORT);
[tcpClient, writer, outStream, tcpOK] = connectTCP(VEHICLE_IP, VEHICLE_PORT, 10);
if ~tcpOK
    warning('[INIT] TCP not connected — DISPLAY-ONLY mode.');
else
    sendCmd(writer, outStream, 'ping');
    fprintf('[INIT] TCP connected.\n\n');
end

%% ── FIGURE WINDOW ────────────────────────────────────────────────────────

if SHOW_VIDEO
    hFig = figure('Name','Sign Detector v10 — Clemson Autonomy Lab', ...
                  'NumberTitle','off','Color','k', ...
                  'Position',[40 40 fW+20 fH+90]);
    hAx  = axes('Parent',hFig,'Position',[0 0.10 1 0.88], ...
                'XTick',[],'YTick',[],'Color','k');
    axis(hAx,'off');
    hImg = image('Parent',hAx,'CData',corrFrame);
    set(hAx,'XLim',[0.5 fW+0.5],'YLim',[0.5 fH+0.5]);
    axis(hAx,'off');

    % Draw ROI boundary so operator can see the search zone
    roiY1 = round(fH * ROI_TOP_FRAC);
    roiY2 = round(fH * ROI_BOTTOM_FRAC);
    roiX1 = round(fW * ROI_LEFT_FRAC);
    roiX2 = round(fW * ROI_RIGHT_FRAC);
    hold(hAx,'on');
    rectangle('Parent',hAx, ...
        'Position',[roiX1 roiY1 roiX2-roiX1 roiY2-roiY1], ...
        'EdgeColor',[0.4 0.4 0.4],'LineStyle','--','LineWidth',1,'Tag','roi');
    hold(hAx,'off');

    hStatus = uicontrol('Parent',hFig,'Style','text', ...
        'Units','normalized','Position',[0 0 1 0.10], ...
        'FontSize',14,'FontWeight','bold', ...
        'ForegroundColor','white','BackgroundColor','black', ...
        'String','Starting...');
    hSub = uicontrol('Parent',hFig,'Style','text', ...
        'Units','normalized','Position',[0 0.10 1 0.04], ...
        'FontSize',9,'ForegroundColor',[0.8 0.8 0.8], ...
        'BackgroundColor','k','String','');
end

%% ── MAIN DETECTION LOOP ──────────────────────────────────────────────────

currentCommand = "none";
noSignFrames   = 0;
loop_n         = 0;
t_start        = tic;

fprintf('[LOOP] Running. Close figure to stop.\n\n');

try
while true
    if SHOW_VIDEO && ~ishandle(hFig), break; end
    loop_n = loop_n + 1;

    %── Capture ───────────────────────────────────────────────────────────
    frame = captureFrame(cam, CAM_W, CAM_H, CAM_MAX_RETRIES);
    if isempty(frame)
        fprintf('[CAM] Skipping frame %d\n', loop_n); continue;
    end
    frame = applyFlip(frame, FLIP_MODE);
    [fH, fW, ~] = size(frame);

    %── Detector ──────────────────────────────────────────────────────────
    detectFrame = enhanceFrame(frame, ENHANCE_CONTRAST);
    [bboxes, scores, labels] = safeDetect(detector, detectFrame, DETECTOR_API);

    %── Verbose ───────────────────────────────────────────────────────────
    if loop_n <= VERBOSE_FRAMES
        if isempty(scores)
            fprintf('[F%03d] 0 detector hits\n', loop_n);
        else
            fprintf('[F%03d] %d hits:\n', loop_n, numel(scores));
            for di = 1:numel(scores)
                mk = ''; if scores(di)>=SCORE_THRESH, mk='  KEEP'; end
                fprintf('       "%s" %.3f%s\n', string(labels(di)), scores(di), mk);
            end
        end
    end

    %── Threshold ─────────────────────────────────────────────────────────
    if ~isempty(scores)
        keep   = scores >= SCORE_THRESH;
        bboxes = bboxes(keep,:);
        scores = scores(keep);
        labels = labels(keep);
    end

    %── Classify ──────────────────────────────────────────────────────────
    sawStop=false; sawSchool=false; bestStopSc=0; bestSlowSc=0;
    for k = 1:numel(scores)
        lbl = lower(strtrim(string(labels(k)))); sc = scores(k);
        if any(arrayfun(@(s) contains(lbl,s), STOP_SUBSTRINGS))
            sawStop=true; bestStopSc=max(bestStopSc,sc);
        elseif any(arrayfun(@(s) contains(lbl,s), SCHOOL_ZONE_SUBSTRINGS))
            sawSchool=true; bestSlowSc=max(bestSlowSc,sc);
        end
    end

    %── Colour Fallback ───────────────────────────────────────────────────
    colorSource = '';
    if USE_COLOR_FALLBACK && ~sawStop && ~sawSchool
        hsv = rgb2hsv(frame);

        % --- ROI crop for colour analysis ---
        r1 = max(1, round(fH*ROI_TOP_FRAC));
        r2 = min(fH, round(fH*ROI_BOTTOM_FRAC));
        roiHSV = hsv(r1:r2, :, :);
        roiH   = roiHSV(:,:,1);
        roiS   = roiHSV(:,:,2);
        roiV   = roiHSV(:,:,3);

        % --- RED mask ---
        redMask = ((roiH>=RED_HUE_LOW1 & roiH<=RED_HUE_HIGH1) | ...
                   (roiH>=RED_HUE_LOW2 & roiH<=RED_HUE_HIGH2)) & ...
                   roiS>=RED_SAT_MIN & roiV>=RED_VAL_MIN;

        % --- YELLOW mask ---
        ylwMask = roiH>=YLW_HUE_LOW & roiH<=YLW_HUE_HIGH & ...
                  roiS>=YLW_SAT_MIN & roiV>=YLW_VAL_MIN;

        % --- Blob analysis ---
        [sawStop,  bestStopSc,  redInfo] = analyseBlobMask( ...
            redMask, RED_BLOB_MIN, RED_BLOB_MAX, ...
            RED_ASPECT_MIN, RED_ASPECT_MAX, ...
            ROI_LEFT_FRAC, ROI_RIGHT_FRAC, fW);

        [sawSchool, bestSlowSc, ylwInfo] = analyseBlobMask( ...
            ylwMask, YLW_BLOB_MIN, YLW_BLOB_MAX, ...
            YLW_ASPECT_MIN, YLW_ASPECT_MAX, ...
            ROI_LEFT_FRAC, ROI_RIGHT_FRAC, fW);

        if loop_n <= VERBOSE_FRAMES
            fprintf('[F%03d] colour red: %s | ylw: %s\n', loop_n, redInfo, ylwInfo);
        end

        if sawStop
            colorSource = sprintf('(colour-red: %s)', redInfo);
        elseif sawSchool
            colorSource = sprintf('(colour-ylw: %s)', ylwInfo);
        end
    end

    %── Command ───────────────────────────────────────────────────────────
    if sawStop
        newCommand   = "stop";
        noSignFrames = 0;
    elseif sawSchool
        newCommand   = "school_zone";
        noSignFrames = 0;
    else
        noSignFrames = noSignFrames + 1;
        newCommand   = currentCommand;
        if noSignFrames >= CLEAR_FRAMES_NEEDED
            newCommand = "clear";
        end
    end

    %── TCP ───────────────────────────────────────────────────────────────
    if newCommand ~= currentCommand
        if tcpOK
            tcpOK = sendCmd(writer, outStream, char(newCommand));
            if ~tcpOK
                [tcpClient,writer,outStream,tcpOK] = connectTCP(VEHICLE_IP,VEHICLE_PORT,3);
                if tcpOK, sendCmd(writer,outStream,char(newCommand)); end
            end
        end
        fprintf('[CMD] %-12s ← %-12s  stop=%.2f school=%.2f  %s\n', ...
            newCommand, currentCommand, bestStopSc, bestSlowSc, colorSource);
        currentCommand = newCommand;
    end

    %── Telemetry ─────────────────────────────────────────────────────────
    if mod(loop_n,30)==0
        fps = loop_n/toc(t_start);
        fprintf('[%05d] FPS=%.1f cmd=%-12s stop=%.2f school=%.2f noSign=%d TCP=%s\n', ...
            loop_n, fps, currentCommand, bestStopSc, bestSlowSc, ...
            noSignFrames, ternStr(tcpOK,'OK','DISC'));
    end

    %── Display ───────────────────────────────────────────────────────────
    if SHOW_VIDEO && ishandle(hFig)
        set(hImg,'CData',frame);
        delete(findobj(hAx,'Tag','det'));
        hold(hAx,'on');

        for k = 1:size(bboxes,1)
            lbl=lower(strtrim(string(labels(k)))); sc=scores(k); bx=bboxes(k,:);
            isS=any(arrayfun(@(s)contains(lbl,s),STOP_SUBSTRINGS));
            isZ=any(arrayfun(@(s)contains(lbl,s),SCHOOL_ZONE_SUBSTRINGS));
            if isS, ec=[1 0.1 0.1]; tc='white';
            elseif isZ, ec=[1 0.85 0]; tc='black';
            else, ec=[0 0.9 0.9]; tc='black'; end
            rectangle('Parent',hAx,'Position',bx,'EdgeColor',ec,'LineWidth',3,'Tag','det');
            fill([bx(1) bx(1)+bx(3) bx(1)+bx(3) bx(1)], ...
                 [bx(2) bx(2) bx(2)-22 bx(2)-22], ...
                 ec,'Parent',hAx,'EdgeColor',ec,'Tag','det');
            text(bx(1)+4,bx(2)-3,sprintf('%s %.0f%%',upper(lbl),sc*100), ...
                 'Parent',hAx,'Color',tc,'FontSize',11,'FontWeight','bold', ...
                 'VerticalAlignment','bottom','Tag','det');
        end
        hold(hAx,'off');

        fps_now = loop_n/toc(t_start);
        tcpStr  = ternStr(tcpOK,'TCP:OK','TCP:DISC');
        switch currentCommand
            case "stop"
                bg=[0.80 0 0]; fg='white';
                msg=sprintf('STOP  |  FPS %.1f  |  %d det  |  %s',fps_now,numel(scores),tcpStr);
            case "school_zone"
                bg=[0.90 0.75 0]; fg='black';
                msg=sprintf('SCHOOL ZONE  |  FPS %.1f  |  %d det  |  %s',fps_now,numel(scores),tcpStr);
            otherwise
                bg=[0 0.45 0]; fg='white';
                msg=sprintf('CLEAR  |  FPS %.1f  |  %d det  |  %s',fps_now,numel(scores),tcpStr);
        end
        set(hStatus,'String',msg,'ForegroundColor',fg,'BackgroundColor',bg);
        set(hSub,'String',colorSource);
        drawnow limitrate;
    end

end % while
catch ME
    fprintf('\n[ERROR] %s  line %d\n  %s\n', ME.stack(1).name, ME.stack(1).line, ME.message);
end

%% ── SHUTDOWN ─────────────────────────────────────────────────────────────
fprintf('[SHUTDOWN] Closing ...\n');
try, sendCmd(writer,outStream,'clear'); pause(0.3); catch, end
try, tcpClient.close(); catch, end
try, clear cam; catch, end
fprintf('[SHUTDOWN] Done.\n');


%% ══════════════════════════════════════════════════════════════════════════
%%  LOCAL FUNCTIONS
%% ══════════════════════════════════════════════════════════════════════════

function [found, conf, info] = analyseBlobMask(mask, blobMin, blobMax, ...
                                                aspMin, aspMax, ...
                                                roiLeftFrac, roiRightFrac, frameW)
    found = false; conf = 0; info = 'none';

    % Label connected components
    CC = bwconncomp(mask, 8);
    if CC.NumObjects == 0, return; end

    stats = regionprops(CC, 'Area','BoundingBox','Centroid');

    bestArea = 0;
    bestAR   = 0;
    bestCX   = 0;

    for i = 1:numel(stats)
        area = stats(i).Area;
        if area < blobMin || area > blobMax, continue; end

        bb  = stats(i).BoundingBox;  % [x y w h]
        bw  = bb(3); bh = bb(4);
        ar  = bw / max(bh, 1);       % width/height ratio

        if ar < aspMin || ar > aspMax, continue; end

        % Check blob centre is in allowed horizontal zone
        cx = stats(i).Centroid(1);
        leftLimit  = frameW * roiLeftFrac;
        rightLimit = frameW * roiRightFrac;
        if cx < leftLimit || cx > rightLimit, continue; end

        % Pick the largest valid blob
        if area > bestArea
            bestArea = area;
            bestAR   = ar;
            bestCX   = cx;
        end
    end

    if bestArea >= blobMin
        found = true;
        conf  = 0.01 + bestArea / 50000;
        info  = sprintf('area=%d ar=%.2f cx=%.0f', bestArea, bestAR, bestCX);
    end
end


function apiMode = probeDetectorAPI(det)
    apiMode  = 'fallback_only';
    dummyImg = uint8(128 * ones(64,64,3));
    try
        [~,sc,~] = detect(det, dummyImg, 'SelectStrongest', true);
        apiMode = 'three_output'; return;
    catch, end
    try
        [~,sc,~] = detect(det, dummyImg);
        apiMode = 'three_output_bare'; return;
    catch, end
    try
        bb = detect(det, dummyImg, 'SelectStrongest', false);
        if ~isempty(bb), classify(det, dummyImg, bb); end
        apiMode = 'one_output_classify'; return;
    catch, end
    try
        bb = detect(det, dummyImg);
        apiMode = 'one_output_classify'; return;
    catch, end
    fprintf('[WARN] Detector probe failed — colour fallback only.\n');
end


function [bboxes,scores,labels] = safeDetect(det, img, apiMode)
    bboxes=zeros(0,4); scores=zeros(0,1); labels={};
    switch apiMode
        case {'three_output','three_output_bare'}
            try
                [bboxes,scores,labels] = detect(det,img,'SelectStrongest',true);
            catch
                try [bboxes,scores,labels] = detect(det,img); catch, return; end
            end
        case 'one_output_classify'
            try
                bboxes = detect(det,img,'SelectStrongest',true);
                if isempty(bboxes), return; end
                [labels,scores] = classify(det,img,bboxes);
            catch
                try
                    bboxes = detect(det,img);
                    if isempty(bboxes), return; end
                    [labels,scores] = classify(det,img,bboxes);
                catch, bboxes=zeros(0,4); return; end
            end
        otherwise
            return;
    end
    scores = double(scores(:));
    if iscell(labels), labels = labels(:); end
end


function out = enhanceFrame(img, enhance)
    if enhance
        try
            out = zeros(size(img),'uint8');
            for c=1:3, out(:,:,c) = imadjust(img(:,:,c)); end
        catch
            out = img;
        end
    else
        out = img;
    end
end


function out = applyFlip(img, mode)
    switch lower(mode)
        case 'rot180',   out = rot90(img,2);
        case 'rot90cw',  out = rot90(img,-1);
        case 'rot90ccw', out = rot90(img,1);
        case 'fliph',    out = fliplr(img);
        case 'flipv',    out = flipud(img);
        otherwise,       out = img;
    end
end


function frame = captureFrame(cam, w, h, maxRetries)
    frame = [];
    for r = 1:maxRetries
        try
            frame = snapshot(cam); return;
        catch camErr
            fprintf('[CAM] Retry %d/%d: %s\n', r, maxRetries, camErr.message);
            pause(0.5);
            try
                cam = webcam("USB Camera");
                cam.Resolution = sprintf('%dx%d',w,h);
            catch, end
        end
    end
end


function [client,writer,outStream,ok] = connectTCP(ip,port,maxAttempts)
    client=[]; writer=[]; outStream=[]; ok=false;
    for a=1:maxAttempts
        try
            client    = java.net.Socket(ip,port);
            client.setTcpNoDelay(true);
            outStream = client.getOutputStream();
            writer    = java.io.PrintWriter(outStream,true);
            ok=true; return;
        catch ex
            fprintf('[TCP] Attempt %d/%d: %s\n',a,maxAttempts,ex.message);
            pause(1);
        end
    end
    warning('[TCP] Cannot connect to %s:%d',ip,port);
end


function ok = sendCmd(writer, outStream, cmd)
    ok=true;
    try
        writer.println(cmd);
        outStream.flush();
    catch ex
        fprintf('[TCP] Send failed: %s\n',ex.message);
        ok=false;
    end
end


function s = ternStr(cond,a,b)
    if cond, s=a; else, s=b; end
end