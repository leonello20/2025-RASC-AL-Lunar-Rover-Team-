function robot = importNBV(type)
    if(type == "xz")
        robot = importrobot("./nbv/urdf/nbv_for_density_xz.urdf");
    else
        robot = importrobot("./nbv/urdf/nbv.urdf");
    end
end