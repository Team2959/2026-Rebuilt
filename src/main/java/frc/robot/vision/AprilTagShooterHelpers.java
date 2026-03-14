// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class AprilTagShooterHelpers {
    private final static double hubX = 4.636;
    private final static double hubY = 4.0;
    private final static double leftBumpX = 4.2;
    private final static double rightBumpX = 2.5;
    private final static double bumpY = 5.5;

    // see spreadsheet calculations:
    // https://docs.google.com/spreadsheets/d/1LeHMqRkg8qOJIvRW7yfcDIsMGJCiFuTLF8awAVgnMac/edit?usp=sharing
    public static double distanceToTarget() {
        var pose3d = alliancePose();
        if (pose3d.getX() <= 0 && pose3d.getY() <= 0)
            return Double.NaN;

        if (isTargetHub(pose3d.getX()))
            return distanceToHub(pose3d);
        return distanceToBump(pose3d);
    }

    public static double turretAngleToTarget(double currentRobotRotation, boolean isShooting) {
        var pose3d = alliancePose();
        // check for valid pose, if not return unusable angle
        if (pose3d.getX() <= 0 && pose3d.getY() <= 0)
            return Double.NaN;

        if (isTargetHub(pose3d.getX()))
            return turretAngleToHub(pose3d, currentRobotRotation);
        if (isShooting)
            return turretAngleToBump(pose3d, currentRobotRotation);

        return Double.NaN;
    }

    private static boolean isTargetHub(double x) {
        return x < 4.4;
    }

    private static boolean isTargetLeftBump(double y) {
        return y > 4;
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
        if (deltaX == 0) {
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
        if (isTargetLeftBump(pose3d.getY()))
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
        if (isTargetLeftBump(pose3d.getY()))
            bumpX = leftBumpX;
        // calculate target turret angle to hub, from spreadsheet calcs
        // B14 = DeltaX
        // B15 = DeltaY
        // =IF(B14 = 0, 0, ATAN(B15/B14))
        var deltaX = deltaXToBump(pose3d, bumpX);
        if (deltaX == 0) {
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

    public static void updateLimelightPose(double turretAngle) {
        // Physical offsets of the camera from the TURRET'S center of rotation
        double x = 0.2032; // meters forward from turret center
        double y = 0.0; // meters left/right
        double z = 0.5207; // meters up from floor

        // Update Limelight with its current position relative to the ROBOT center
        // Note: The 'yaw' parameter here is your turret's current angle
        LimelightHelpers.setCameraPose_RobotSpace("limelight",
                x, y, z,
                0, 0, turretAngle);
    }

    public static void updateRobotOrientation(double yaw, double yawRate) {
        LimelightHelpers.SetRobotOrientation("limelight",
                yaw, yawRate, 0, 0, 0, 0);
    }

    // what do you do if the limelight doesn't see an april tag????
    private static PoseEstimate alliancePoseMt2() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
        else
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }

    private static Translation2d targeTranslation2d(PoseEstimate mt2, boolean isShooting) {
        if (isTargetHub(mt2.pose.getX())) {
            // Field coordinates for the center of the Hub (example values)
            return new Translation2d(hubX, hubY);
        }

        if (isShooting) {
            var bumpX = rightBumpX;
            if (isTargetLeftBump(mt2.pose.getY()))
                bumpX = leftBumpX;
            return new Translation2d(bumpX, bumpY);
        }

        return Translation2d.kZero;
    }

    public static double mt2TargetAngle(boolean isShooting) {
        // 2. Get the field-space pose estimate
        var mt2 = alliancePoseMt2();

        if (mt2.tagCount > 0) {
            Translation2d target = targeTranslation2d(mt2, isShooting);

            if (target == Translation2d.kZero)
                return Double.NaN;

            // Calculate angle from robot to hub
            Rotation2d fieldAngleToHub = target.minus(mt2.pose.getTranslation()).getAngle();

            // Convert to the angle the turret needs to be at relative to the robot
            return fieldAngleToHub.minus(mt2.pose.getRotation()).getDegrees();
        }
        return Double.NaN;
    }

    public static double mt2DistanceToTaget(boolean isShooting) {
        var mt2 = alliancePoseMt2();
        if (mt2.tagCount > 0) {
            Translation2d target = targeTranslation2d(mt2, isShooting);

            if (target == Translation2d.kZero)
                return Double.NaN;

            return mt2.pose.getTranslation().getDistance(target);
        }
        return Double.NaN;
    }
}
