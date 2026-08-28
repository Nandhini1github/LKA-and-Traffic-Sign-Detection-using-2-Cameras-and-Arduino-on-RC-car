%% =========================================================
%  PART 1: Train CIFAR10Net from CIFAR-10 Batch Files
% ==========================================================

clc; clear; close all;

%% STEP 1 — Set your CIFAR-10 folder path
% *** UPDATE THIS PATH TO MATCH YOUR COMPUTER ***
cifarFolder = 'C:\Users\aloks\Desktop\ADT_FINALPROJECT\robocar\cifar-10-batches-mat';

if ~isfolder(cifarFolder)
    error('Folder not found. Please update cifarFolder path on line 10.');
end
fprintf('CIFAR-10 folder found.\n');

%% STEP 2 — Load all 5 training batches + test batch

fprintf('Loading batch files...\n');

function [images, labels] = loadCIFARBatch(filepath)
    raw    = load(filepath);
    images = raw.data;
    labels = raw.labels + 1;   % shift from 0-indexed to 1-indexed
end

[img1, lbl1] = loadCIFARBatch(fullfile(cifarFolder, 'data_batch_1'));
[img2, lbl2] = loadCIFARBatch(fullfile(cifarFolder, 'data_batch_2'));
[img3, lbl3] = loadCIFARBatch(fullfile(cifarFolder, 'data_batch_3'));
[img4, lbl4] = loadCIFARBatch(fullfile(cifarFolder, 'data_batch_4'));
[img5, lbl5] = loadCIFARBatch(fullfile(cifarFolder, 'data_batch_5'));
[imgTest, lblTest] = loadCIFARBatch(fullfile(cifarFolder, 'test_batch'));

fprintf('All batch files loaded.\n');

%% STEP 3 — Combine and reshape to 32x32x3

fprintf('Reshaping images...\n');

allImages = [img1; img2; img3; img4; img5];
allLabels = [lbl1; lbl2; lbl3; lbl4; lbl5];
numTrain  = size(allImages, 1);

XTrain = zeros(32, 32, 3, numTrain, 'uint8');
for i = 1:numTrain
    r = reshape(allImages(i, 1:1024),    [32 32])';
    g = reshape(allImages(i, 1025:2048), [32 32])';
    b = reshape(allImages(i, 2049:3072), [32 32])';
    XTrain(:,:,:,i) = cat(3, r, g, b);
end
YTrain = categorical(allLabels);

numTest = size(imgTest, 1);
XTest   = zeros(32, 32, 3, numTest, 'uint8');
for i = 1:numTest
    r = reshape(imgTest(i, 1:1024),    [32 32])';
    g = reshape(imgTest(i, 1025:2048), [32 32])';
    b = reshape(imgTest(i, 2049:3072), [32 32])';
    XTest(:,:,:,i) = cat(3, r, g, b);
end
YTest = categorical(lblTest);

fprintf('Reshaping complete. XTrain: %s | XTest: %s\n', ...
    mat2str(size(XTrain)), mat2str(size(XTest)));

%% STEP 4 — Define CIFAR10Net architecture

fprintf('Defining network architecture...\n');

layers = [
    imageInputLayer([32 32 3], 'Name', 'input', ...
                    'Normalization', 'zerocenter')

    % Conv Block 1
    convolution2dLayer(5, 32, 'Padding', 'same', 'Name', 'conv1')
    batchNormalizationLayer('Name', 'bn1')
    leakyReluLayer(0.1, 'Name', 'lrelu1')
    maxPooling2dLayer(2, 'Stride', 2, 'Name', 'pool1')

    % Conv Block 2
    convolution2dLayer(5, 64, 'Padding', 'same', 'Name', 'conv2')
    batchNormalizationLayer('Name', 'bn2')
    leakyReluLayer(0.1, 'Name', 'lrelu2')
    maxPooling2dLayer(2, 'Stride', 2, 'Name', 'pool2')

    % Fully Connected 1
    fullyConnectedLayer(256, 'Name', 'fc1')
    dropoutLayer(0.5, 'Name', 'drop1')
    sigmoidLayer('Name', 'sig1')

    % Fully Connected 2
    fullyConnectedLayer(10, 'Name', 'fc2')
    dropoutLayer(0.5, 'Name', 'drop2')
    sigmoidLayer('Name', 'sig2')

    % Output
    softmaxLayer('Name', 'softmax')
    classificationLayer('Name', 'output')
];

fprintf('Architecture defined.\n');

%% STEP 5 — Training options
% Change 'cpu' to 'gpu' if you have an Nvidia GPU

options = trainingOptions('sgdm', ...
    'MiniBatchSize',        128, ...
    'MaxEpochs',            50, ...
    'InitialLearnRate',     0.01, ...
    'Momentum',             0.9, ...
    'L2Regularization',     1e-4, ...
    'LearnRateSchedule',    'piecewise', ...
    'LearnRateDropFactor',  0.1, ...
    'LearnRateDropPeriod',  20, ...
    'Shuffle',              'every-epoch', ...
    'ValidationData',       {XTest, YTest}, ...
    'ValidationFrequency',  200, ...
    'Verbose',              true, ...
    'Plots',                'training-progress', ...
    'ExecutionEnvironment', 'cpu');

%% STEP 6 — Train CIFAR10Net
% Expect 30 min to 2 hours on CPU

fprintf('\nStarting training... (30 min - 2 hrs on CPU)\n');
fprintf('Watch the Training Progress window.\n\n');

cifar10Net = trainNetwork(XTrain, YTrain, layers, options);

fprintf('Training complete!\n');

%% STEP 7 — Check accuracy

YPred    = classify(cifar10Net, XTest);
accuracy = sum(YPred == YTest) / numel(YTest) * 100;
fprintf('Test Accuracy: %.2f%%\n', accuracy);

%% STEP 8 — Save

save('cifar10Net.mat', 'cifar10Net');
fprintf('Saved to cifar10Net.mat in: %s\n', pwd);
fprintf('You are ready for Part 2 (R-CNN sign detection)!\n');