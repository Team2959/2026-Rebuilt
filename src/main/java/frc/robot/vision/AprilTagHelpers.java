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
        var pose3d = alliancePose();

        // calculate distance from robot to hub, from spreadsheet calcs
        // B14 = DeltaX
        // B15 = DeltaY
        // =SQRT(B14*B14+B15*B15)
        var deltaX = deltaXToHub(pose3d);
        var deltaY = deltaYToHub(pose3d);

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public static double turretAngleToHub(double currentRobotRotation) {
        var pose3d = alliancePose();

        // calculate target turret angle to hub, from spreadsheet calcs
        // B14 = DeltaX
        // B15 = DeltaY
        // =IF(B14 = 0, 0, ATAN(B15/B14))
        var deltaX = deltaXToHub(pose3d);
        if (deltaX == 0)
            return 0;

        var fieldAngle = Math.atan(deltaYToHub(pose3d) / deltaX);

        return Math.toDegrees(fieldAngle) - currentRobotRotation;
    }

    // need to get x and y forom limelight
    private static double deltaXToHub(Pose3d pose3d) {
        return hubX - pose3d.getX();
    }

    private static double deltaYToHub(Pose3d pose3d) {
        return hubY - pose3d.getY();
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
