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
    private final static double leftBumpX = 4.2;
    private final static double rightBumpX = 2.5;
    private final static double bumpY = 5.5;

    // see spreadsheet calculations: https://docs.google.com/spreadsheets/d/1LeHMqRkg8qOJIvRW7yfcDIsMGJCiFuTLF8awAVgnMac/edit?usp=sharing
    public static double distanceToTarget()
    {
        var pose3d = alliancePose();
        if (pose3d.getX() <= 0 && pose3d.getY() <= 0)
            return Double.NaN;
        
        if (isTargetHub(pose3d))
            return distanceToHub(pose3d);
        return distanceToBump(pose3d);
    }

    public static double turretAngleToTarget(double currentRobotRotation) {
        var pose3d = alliancePose();
        // check for valid pose, if not return unusable angle
        if (pose3d.getX() <= 0 && pose3d.getY() <= 0)
            return Double.NaN;
        
        if (isTargetHub(pose3d))
            return turretAngleToHub(pose3d, currentRobotRotation);
        return turretAngleToBump(pose3d, currentRobotRotation);
    }

    private static boolean isTargetHub(Pose3d pose3d){
        return pose3d.getX() < 4.4;
    }

    private static boolean isTargetLeftBump(Pose3d pose3d){
        return pose3d.getY() > 4;
    }

    private static double distanceToHub(Pose3d pose3d) {
        // calculate distance from robot to hub, from spreadsheet calcs
        // B14 = DeltaX
        // B15 = DeltaY
        // =SQRT(B14*B14+B15*B15)
        var deltaX = deltaXToHub(pose3d);
        var deltaY = deltaYToHub(pose3d);

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    private static double turretAngleToHub(Pose3d pose3d, double currentRobotRotation) {
        // calculate target turret angle to hub, from spreadsheet calcs
        // B14 = DeltaX
        // B15 = DeltaY
        // =IF(B14 = 0, 0, ATAN(B15/B14))
        var deltaX = deltaXToHub(pose3d);
        if (deltaX == 0)
        {
            if (deltaYToHub(pose3d) < 0)
                return -90;
            return 90;
        }

        var theta = Math.atan(deltaYToHub(pose3d) / deltaX);

        return Math.toDegrees(theta) - currentRobotRotation;
    }

    private static double deltaXToHub(Pose3d pose3d) {
        return hubX - pose3d.getX();
    }

    private static double deltaYToHub(Pose3d pose3d) {
        return hubY - pose3d.getY();
    }

    private static double distanceToBump(Pose3d pose3d) {
        var bumpX = rightBumpX;
        if (isTargetLeftBump(pose3d))
            bumpX = leftBumpX;
        // calculate distance from robot to bump, from spreadsheet calcs
        // B14 = DeltaX
        // B15 = DeltaY
        // =SQRT(B14*B14+B15*B15)
        var deltaX = deltaXToBump(pose3d, bumpX);
        var deltaY = deltaYToBump(pose3d);

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    private static double deltaXToBump(Pose3d pose3d, double bumpX) {
        return pose3d.getX() - bumpX;
    }

    private static double deltaYToBump(Pose3d pose3d) {
        return pose3d.getY() - bumpY;
    }

    private static double turretAngleToBump(Pose3d pose3d, double currentRobotRotation) {
        var bumpX = rightBumpX;
        if (isTargetLeftBump(pose3d))
            bumpX = leftBumpX;
        // calculate target turret angle to hub, from spreadsheet calcs
        // B14 = DeltaX
        // B15 = DeltaY
        // =IF(B14 = 0, 0, ATAN(B15/B14))
        var deltaX = deltaXToBump(pose3d, bumpX);
        if (deltaX == 0)
        {
            if (deltaYToBump(pose3d) < 0)
                return -90;
            return 90;
        }

        var theta = Math.atan(deltaYToBump(pose3d) / deltaX);
        var robotZeroAngleDegress = 180 + Math.toDegrees(theta);

        return robotZeroAngleDegress - currentRobotRotation;
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
