function pcd2ply(dir, fname)
    pcd_file = append(fname, '.pcd');
    ply_file = append(fname, '.ply');
    path = fullfile(dir, pcd_file);
    cloud = pcread(path);
    pcwrite(cloud, fullfile(dir, ply_file));