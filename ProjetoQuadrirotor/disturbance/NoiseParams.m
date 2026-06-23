function noiseParams = NoiseParams()

noiseParams.r.whiteNoise = [
    0.02;
    0.02;
    0.02
];

noiseParams.v.whiteNoise = [
    0.02;
    0.02;
    0.02
];

noiseParams.eta.whiteNoise = deg2rad([
    0.5;
    0.5;
    1.0
]);

noiseParams.omega.whiteNoise = deg2rad([
    0.2;
    0.2;
    0.2
]);

end