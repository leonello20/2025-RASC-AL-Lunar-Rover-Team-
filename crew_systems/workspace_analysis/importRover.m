function robot = importRover(rbt)
    if(rbt == "nbv")
        robot = importrobot("./rover_crane/urdf/rover_crane_nbv.urdf");
    else if(rbt == "sawyer")
        robot = importrobot("./rover_crane/urdf/rover_crane_sawyer.urdf");
    else
        robot = importrobot("./rover_crane/urdf/rover_crane.urdf");
    end

end