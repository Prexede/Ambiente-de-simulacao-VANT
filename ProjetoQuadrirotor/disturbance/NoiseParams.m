function noiseParams = NoiseParams()

noiseParams.r.whiteNoise = [
    0.2;
    0.01;
    0.01
];

noiseParams.v.whiteNoise = [
    0.1;
    0.1;
    0.1
];

noiseParams.eta.whiteNoise = deg2rad([
    0.5;
    0.5;
    0.5
]);

noiseParams.omega.whiteNoise = deg2rad([
    0.2;
    0.2;
    0.2
]);

end