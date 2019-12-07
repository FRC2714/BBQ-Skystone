package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.localization.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class LinearPathBuilder {
    private int pathIdx = 0;
    private int poseIdx = 0;

    private List<List<Pose2d>> paths;
    public LinearPathBuilder() {
        this.paths = new ArrayList<>();
    }

    public void addPath(List<Pose2d> poses) {
        this.paths.add(poses);

    }
}
