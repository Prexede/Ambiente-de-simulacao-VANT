function noiseParams = NoiseParams()

noiseParams.r.whiteNoise = [
    0.02;
    0.02;
    0.6
];

noiseParams.v.whiteNoise = [
    0.02;
    0.02;
    0.02
];

noiseParams.eta.whiteNoise = deg2rad([
    0.02;
    0.02;
    0.02
]);

noiseParams.omega.whiteNoise = deg2rad([
    0.02;
    0.02;
    0.02
]);

end