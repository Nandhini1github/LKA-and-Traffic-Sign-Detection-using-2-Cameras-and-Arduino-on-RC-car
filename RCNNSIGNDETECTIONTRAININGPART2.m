%% =========================================================
%  PART 2: R-CNN Sign Detection - Stop & School Signs
%  Robo Car Road Sign Recognition
% ==========================================================
%  REQUIREMENTS BEFORE RUNNING:
%    - gTruth must be in your workspace (from Image Labeler)
%    - cifar10Net.mat must be saved from Part 1
%    - Deep Learning Toolbox
%    - Computer Vision Toolbox
% ==========================================================

clc; close all;
% NOTE: Do NOT put "clear" here — we need gTruth from workspace!

fprintf('===========================================\n');
fprintf('  PART 2: R-CNN Sign Detection Training\n');
fprintf('  Signs: Stop + School\n');
fprintf('===========================================\n');

%% STEP 1 — Check gTruth exists in workspace

if ~exist('gTruth', 'var')
    error(['gTruth not found in workspace!\n' ...
           'Please run Image Labeler, label your signs,\n' ...
           'and export labels to workspace first.']);
end
fprintf('gTruth found in workspace.\n');

%% STEP 2 — Convert gTruth to training data table

fprintf('\nConverting gTruth to training data...\n');

trainingData = objectDetectorTrainingData(gTruth);

fprintf('Training data created!\n');
fprintf('Total labeled images: %d\n', height(trainingData));
fprintf('\nPreview of training data table:\n');
disp(head(trainingData, 5));

%% STEP 3 — Load cifar10Net from Part 1

fprintf('Loading cifar10Net from Part 1...\n');

if ~exist('cifar10Net', 'var')
    % Try to load from saved file
    if isfile('cifar10Net.mat')
        load('cifar10Net.mat');
        fprintf('cifar10Net loaded from cifar10Net.mat\n');
    else
        error(['cifar10Net not found!\n' ...
               'Make sure cifar10Net.mat is in your current MATLAB folder.\n' ...
               'Run Part 1 first to generate it.']);
    end
else
    fprintf('cifar10Net already in workspace.\n');
end

%% STEP 4 — Set R-CNN training options
% Using CPU as selected — change to 'gpu' if you upgrade hardware

fprintf('\nSetting R-CNN training options...\n');

rcnnOptions = trainingOptions('sgdm', ...
    'MiniBatchSize',        128, ...
    'InitialLearnRate',     1e-3, ...
    'Momentum',             0.9, ...
    'LearnRateSchedule',    'piecewise', ...
    'LearnRateDropFactor',  0.1, ...
    'LearnRateDropPeriod',  100, ...
    'MaxEpochs',            100, ...
    'Shuffle',              'every-epoch', ...
    'Verbose',              true, ...
    'Plots',                'training-progress', ...
    'ExecutionEnvironment', 'cpu');

fprintf('Training options set.\n');

%% STEP 5 — Train the R-CNN object detector
% NegativeOverlapRange [0 0.3]  → IoU below 0.3 = background
% PositiveOverlapRange [0.5 1]  → IoU above 0.5 = sign detected

fprintf('\nStarting R-CNN training...\n');
fprintf('(This may take several minutes to hours on CPU)\n');
fprintf('Watch the Training Progress window.\n\n');

rcnn = trainRCNNObjectDetector(trainingData, cifar10Net, rcnnOptions, ...
    'NegativeOverlapRange', [0 0.3], ...
    'PositiveOverlapRange', [0.5 1]);

fprintf('\nR-CNN training complete!\n');

%% STEP 6 — Save the trained R-CNN detector

save('roboCarSignDetector.mat', 'rcnn');
fprintf('R-CNN detector saved to: roboCarSignDetector.mat\n');
fprintf('Location: %s\n', fullfile(pwd, 'roboCarSignDetector.mat'));

%% STEP 7 — Test on a single robo car image
% *** UPDATE testImagePath to one of your robo car photos ***

fprintf('\nTesting R-CNN detector on a sample image...\n');

testImagePath = 'C:\Users\aloks\Desktop\ADT_FINALPROJECT\robocar\sign_image_school_zone_00027.jpg';

if ~isfile(testImagePath)
    fprintf('\nWARNING: Test image not found.\n');
    fprintf('Update testImagePath on line 94 to one of your robo car photos.\n');
    fprintf('Then re-run just STEP 7.\n');
else
    % Load test image
    testImage = imread(testImagePath);

    % Run detection
    [bboxes, score, label] = detect(rcnn, testImage, 'MiniBatchSize', 128);

    if isempty(bboxes)
        fprintf('No signs detected in the test image.\n');
        fprintf('Tips:\n');
        fprintf('  - Add more training images (aim for 60-80 per sign)\n');
        fprintf('  - Make sure bounding boxes were drawn tightly\n');
        fprintf('  - Try increasing MaxEpochs to 150\n');
    else
        % Get best detection (highest confidence)
        [bestScore, idx] = max(score);
        bbox             = bboxes(idx, :);
        annotation       = sprintf('%s (%.1f%%)', label(idx), bestScore * 100);

        % Display result
        outputImage = insertObjectAnnotation(testImage, 'rectangle', ...
                        bbox, annotation, ...
                        'FontSize',        18, ...
                        'Color',           'yellow', ...
                        'TextBoxOpacity',  0.7);

        figure('Name', 'Robo Car Sign Detection Result');
        imshow(outputImage);
        title(sprintf('Detected: %s | Confidence: %.1f%%', ...
              label(idx), bestScore * 100));

        % Print results to command window
        fprintf('\n--- Detection Result ---\n');
        fprintf('Sign detected : %s\n',   label(idx));
        fprintf('Confidence    : %.1f%%\n', bestScore * 100);
        fprintf('Bounding box  : x=%d, y=%d, width=%d, height=%d\n', ...
                bbox(1), bbox(2), bbox(3), bbox(4));

        % Show all detections if more than one found
        if length(score) > 1
            fprintf('\nAll detections:\n');
            for i = 1:length(score)
                fprintf('  %d. %s - %.1f%%\n', i, label(i), score(i)*100);
            end
        end
    end
end

fprintf('\n===========================================\n');
fprintf('  PART 2 COMPLETE!\n');
fprintf('  Saved: roboCarSignDetector.mat\n');
fprintf('  Use this .mat file for your robo car demo.\n');
fprintf('===========================================\n');