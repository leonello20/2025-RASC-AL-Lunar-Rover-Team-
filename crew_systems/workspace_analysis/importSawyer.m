function robot = importSawyer(type)
    if(type == "xz")
        robot = importrobot("./sawyer/urdf/sawyer_for_density_xz.urdf");
    else
        robot = importrobot("./sawyer/urdf/sawyer.urdf");
    end
end