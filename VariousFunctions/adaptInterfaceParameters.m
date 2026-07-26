function adaptInterfaceParameters(inputModelPath, outputModelPath)

%% Import OpenSim Libraries
import org.opensim.modeling.*

%% Define Variables
% default translational parameters
k_trans = [5000 500 5000];
d_trans = [500 50 500];

% default rotational parameters
k_rot = [100 30 100];
d_rot = [4 1.5 4];

% allowed change
delta = 0.5;

% translational limits
k_trans_min = k_trans - delta*k_trans;
k_trans_max = k_trans + delta*k_trans;

d_trans_min = d_trans - delta*d_trans;
d_trans_max = d_trans + delta*d_trans;

% rotational limits
k_rot_min = k_rot - delta*k_rot;
k_rot_max = k_rot + delta*k_rot;

d_rot_min = d_rot - delta*d_rot;
d_rot_max = d_rot + delta*d_rot;

%% Load Model
model = Model(inputModelPath);

%% Find Bushing Force
ForceSet = model.getForceSet();
Nforces = ForceSet.getSize();

bushing_names = [];
for i = 1:Nforces
    force = ForceSet.get(i-1);
    if(string(force.getConcreteClassName()) == "BushingForce")
        bushing_names = [bushing_names string(force.getName())];
    end
end

%% Upper Bushing Force
% find upper bushings
isUpper = contains(bushing_names, "upper");
bushing_names_upper = bushing_names(isUpper);
Nupper = length(bushing_names_upper);

% generate new random stiffness parameters
k_trans_xz_upper = k_trans_min(1) + rand()*(k_trans_max(1) - k_trans_min(1));
k_trans_y_upper = k_trans_min(2) + rand()*(k_trans_max(2) - k_trans_min(2));


k_rot_xz_upper = k_rot_min(1) + rand()*(k_rot_max(1) - k_rot_min(1));
k_rot_y_upper = k_rot_min(2) + rand()*(k_rot_max(2) - k_rot_min(2));

% generate new random damping parameters
d_trans_xz_upper = d_trans_min(1) + rand()*(d_trans_max(1) - d_trans_min(1));
d_trans_y_upper = d_trans_min(2) + rand()*(d_trans_max(2) - d_trans_min(2));

d_rot_xz_upper = d_rot_min(1) + rand()*(d_rot_max(1) - d_rot_min(1));
d_rot_y_upper = d_rot_min(2) + rand()*(d_rot_max(2) - d_rot_min(2));

% loop over upper bushings
for i = 1:Nupper
    bushingForce = BushingForce.safeDownCast(ForceSet.get(bushing_names_upper(i)));

    % assign new parameters
    bushingForce.set_translational_stiffness(Vec3(k_trans_xz_upper, k_trans_y_upper, k_trans_xz_upper));
    bushingForce.set_translational_damping(Vec3(d_trans_xz_upper, d_trans_y_upper, d_trans_xz_upper));
    bushingForce.set_rotational_stiffness(Vec3(k_rot_xz_upper, k_rot_y_upper, k_rot_xz_upper));
    bushingForce.set_rotational_damping(Vec3(d_rot_xz_upper, d_rot_y_upper, d_rot_xz_upper));
end

%% Lower Bushing Force
% find lower bushings
isLower = contains(bushing_names, "lower");
bushing_names_lower = bushing_names(isLower);
Nupper = length(bushing_names_lower);

% generate new random stiffness parameters
k_trans_xz_lower = k_trans_min(1) + rand()*(k_trans_max(1) - k_trans_min(1));
k_trans_y_lower = k_trans_min(2) + rand()*(k_trans_max(2) - k_trans_min(2));


k_rot_xz_lower = k_rot_min(1) + rand()*(k_rot_max(1) - k_rot_min(1));
k_rot_y_lower = k_rot_min(2) + rand()*(k_rot_max(2) - k_rot_min(2));

% generate new random damping parameters
d_trans_xz_lower = d_trans_min(1) + rand()*(d_trans_max(1) - d_trans_min(1));
d_trans_y_lower = d_trans_min(2) + rand()*(d_trans_max(2) - d_trans_min(2));

d_rot_xz_lower = d_rot_min(1) + rand()*(d_rot_max(1) - d_rot_min(1));
d_rot_y_lower = d_rot_min(2) + rand()*(d_rot_max(2) - d_rot_min(2));

% loop over upper bushings
for i = 1:Nupper
    bushingForce = BushingForce.safeDownCast(ForceSet.get(bushing_names_lower(i)));

    % assign new parameters
    bushingForce.set_translational_stiffness(Vec3(k_trans_xz_lower, k_trans_y_lower, k_trans_xz_lower));
    bushingForce.set_translational_damping(Vec3(d_trans_xz_lower, d_trans_y_lower, d_trans_xz_lower));
    bushingForce.set_rotational_stiffness(Vec3(k_rot_xz_lower, k_rot_y_lower, k_rot_xz_lower));
    bushingForce.set_rotational_damping(Vec3(d_rot_xz_lower, d_rot_y_lower, d_rot_xz_lower));
end

%% Save Model
try
    model.finalizeConnections();
    model.print(outputModelPath);
    fprintf('Written model %s at %s\n', outputModelPath, datetime('now','format','HH:mm:ss'))
catch ME
    disp("ERROR writing model");
    disp(getReport(ME));
end

%% Debug Plot
% t = tiledlayout(2,4);
% 
% nexttile
% hold on
% plot(1, k_trans_xz_upper, "b*")
% plot(1, k_trans_xz_lower, "b*")
% ylim([k_trans_min(1), k_trans_max(1)])
% xticklabels([])
% ylabel("[N/m]","FontWeight","bold")
% title(["Translational"; "Stiffness XZ"])
% 
% nexttile
% hold on
% plot(1, k_rot_xz_upper, "b*")
% plot(1, k_rot_xz_lower, "b*")
% ylim([k_rot_min(1), k_rot_max(1)])
% xticklabels([])
% ylabel("[Nm/rad]","FontWeight","bold")
% title(["Rotation"; "Stiffness XZ"])
% 
% nexttile
% hold on
% plot(1, d_trans_xz_upper, "b*")
% plot(1, d_trans_xz_lower, "b*")
% ylim([d_trans_min(1), d_trans_max(1)])
% xticklabels([])
% ylabel("[N/m/s]","FontWeight","bold")
% title(["Translational"; "Damping XZ"])
% 
% nexttile
% hold on
% plot(1, d_rot_xz_upper, "b*")
% plot(1, d_rot_xz_lower, "b*")
% ylim([d_rot_min(1), d_rot_max(1)])
% xticklabels([])
% ylabel("[Nm/rad/s]","FontWeight","bold")
% title(["Rotational"; "Damping XZ"])
% 
% nexttile
% hold on
% plot(1, k_trans_y_upper, "b*")
% plot(1, k_trans_y_lower, "b*")
% ylim([k_trans_min(2), k_trans_max(2)])
% xticklabels([])
% ylabel("[N/m]","FontWeight","bold")
% title(["Translational"; "Stiffness Y"])
% 
% nexttile
% hold on
% plot(1, k_rot_y_upper, "b*")
% plot(1, k_rot_y_lower, "b*")
% ylim([k_rot_min(2), k_rot_max(2)])
% xticklabels([])
% ylabel("[Nm/rad]","FontWeight","bold")
% title(["Rotation"; "Stiffness Y"])
% 
% nexttile
% hold on
% plot(1, d_trans_y_upper, "b*")
% plot(1, d_trans_y_lower, "b*")
% ylim([d_trans_min(2), d_trans_max(2)])
% xticklabels([])
% ylabel("[N/m/s]","FontWeight","bold")
% title(["Translational"; "Damping Y"])
% 
% nexttile
% hold on
% plot(1, d_rot_y_upper, "b*")
% plot(1, d_rot_y_lower, "b*")
% ylim([d_rot_min(2), d_rot_max(2)])
% xticklabels([])
% ylabel("[Nm/rad/s]","FontWeight","bold")
% title(["Rotational"; "Damping Y"])

end