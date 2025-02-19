function robot = importDyma(rbt)
    if(rbt == "xz")
        robot = importrobot("./dymaflight/urdf/dymaflight_for_density_xz.urdf");
    else
        robot = importrobot("./dymaflight/urdf/dymaflight.urdf");
    end
end