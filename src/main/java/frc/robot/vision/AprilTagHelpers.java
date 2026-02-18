// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class AprilTagHelpers {
    private final static double hubX = 4.636;
    private final static double hubY = 4.0;

    public static double distanceToHub() {
        // calculate distance from robot to hub
        // =SQRT(B14*B14+B15*B15)
        var deltaX = deltaXToHub();
        var deltaY = deltaYToHub();

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public static double turretAngleToHub(double currentRobotRotation) {
        // B14 = DeltaX
        // B15 = DeltaY
        // =IF(B14 = 0, 0, ATAN(B15/B14))
        var deltaX = deltaXToHub();
        if (deltaX == 0)
            return 0;

        var fieldAngle = Math.atan(deltaYToHub() / deltaX);

        return Math.toDegrees(fieldAngle) - currentRobotRotation;
    }

    // need to get x and y forom limelight
    private static double deltaXToHub() {
        var pose3d = alliancePose();
        var robotX = pose3d.getX();
        return hubX - robotX;
    }

    private static double deltaYToHub() {
        var pose3d = alliancePose();
        var robotY = pose3d.getY();
        return hubY - robotY;
    }

    // what do you do if the limelight doesn't see an april tag????
    private static Pose3d alliancePose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            return LimelightHelpers.getBotPose3d_wpiRed("limelight");
        else
            return LimelightHelpers.getBotPose3d_wpiBlue("limelight");
    }
}
