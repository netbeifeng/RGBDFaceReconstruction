% README
% Run `saveBFMLandmarks('Landmarks68_BFM.anl')` on the command window
% Landmarks are saved in 'BFM_landmarks.csv'.
% Note: Change `bfm_path` to point to BFM. 2009 version is useds here.
function landmarks = saveBFMLandmarks(filename)

    % Download Basel Face Model (2009) here:
    % https://faces.dmi.unibas.ch/bfm/index.php?nav=1-1-0&id=details
    bfm_path = './01_MorphableModel.mat';
   
    landmarks = zeros(68,3);

    idx = readLandmarks(filename);
    face = load(bfm_path);
    shape = reshape(face.shapeMU, 3, 53490);
    shape = shape.';
    x = shape(:, 1);
    y = shape(:, 2);
    z = shape(:, 3);
    scatter3(x, y, z, 2, 'filled');
    hold on;
    for i = 1:68
        scatter3(x(idx(i)), y(idx(i)), z(idx(i)),10, 'r');
        landmarks(i, :) = [x(idx(i)), y(idx(i)), z(idx(i))];
    end
    % save landmarks landmarks
    csvwrite('BFM_landmarks.csv', landmarks)