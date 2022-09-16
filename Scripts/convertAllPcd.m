
function convertAllPcd(dir)
    id_base = 'ID_';
    expression_dir = ["01_HeadScan", "02_Anger", "03_Disgust", "04_Fear", "05_Happiness", "06_Sadness", "07_Surprise", "08_Random"];
    pcd_dir = 'FaceRaw';
    fname_base = 'Face_Raw_';
    
    total_id = 40;
    total_expression = 8;
    
    for i = 1:total_id
        id = append(id_base, num2str(i,'%03d'));
        for j = 1:total_expression
            path = fullfile(dir, id, expression_dir(j), pcd_dir);
            disp(path);
            n = 30;
            if j == 1
                n = 90;
            elseif j == 8
                 n = 150;
            end
            for k = 0:n-1
                fname = append(fname_base, num2str(k,'%06d'));
                % disp(fname)
                pcd2ply(path, fname);
            end
        end
    end

convertAllPcd("")    
