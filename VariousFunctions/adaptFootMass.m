% --------------------------------------------------------------------------
% adaptFootMass
%   Change the mass of the foot, including the calcaneus, talus and toes,
%   by a given factor.
%
% Original author: Menthy Denayer
% Original date: 06/January/2026
% --------------------------------------------------------------------------

function adaptFootMass(factor, params, modelFile)

%% Import Libraries
import org.opensim.modeling.*

%% Load Model
adaptedModel = Model(params.modelPath);

%% Initialize Model Info
FootBodyNames = ["calcn_r","calcn_l","talus_r","talus_l","toes_r","toes_l"];
NFootBodies = length(FootBodyNames);
Bodies = adaptedModel.getBodySet();

%% Adapt Foot Mass
for b = 1:NFootBodies
    body = Bodies.get(FootBodyNames(b));
    body.setMass(body.getMass()*factor)
    updateInertia(factor, body)
end

adaptedModel.finalizeConnections();

try
    adaptedModel.initSystem();
    adaptedModel.print(modelFile);
    fprintf('Written model %s at %s\n', modelFile, datetime('now','format','HH:mm:ss'))
%     fprintf('Model parameters are %0.2f', coeffs0);
catch ME
    disp("ERROR writing model:");
    disp(params.adaptedModelPath);
    disp(getReport(ME));
end

%% Helper Functions
function updateInertia(factor, body)
    newInertia = [body.getInertia().getMoments().getAsMat()*factor, zeros(3,1)];
    body.setInertia(org.opensim.modeling.Inertia(newInertia(1),newInertia(2),newInertia(3),newInertia(4),newInertia(5),newInertia(6)))
end

end