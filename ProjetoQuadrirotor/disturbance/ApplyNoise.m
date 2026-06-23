function measuredValue = ApplyNoise(stateName, trueValue, noiseParams)

measuredValue = trueValue;

if ~noiseParams.enable
    return;
end

stateNoise = noiseParams.(stateName);

if ~stateNoise.enable
    return;
end

switch noiseParams.disturbanceType

    case "whiteNoise"
        measuredValue = trueValue + stateNoise.whiteNoise .* randn(size(trueValue));

end

end