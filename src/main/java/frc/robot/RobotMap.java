// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotMap {

    // Swerve Motor Assemblies - SparkMax
    // Drive motor CAN == module #
    // Steer motor CAN == module # + 10
    public static final int kFrontLeftModule = 4;
    public static final int kBackLeftModule = 1;
    public static final int kBackRigvhtModule = 2;
    public static final int kFrontRightModule = 3;

    // CAN motor addresses
        // SparkMax

    // Analog Input addresses
    public static final int kFrontLeftAnalogInput = 0;
    public static final int kBackLeftAnalogInput = 1;
    public static final int kBackRightAnalogInput = 2;
    public static final int kFrontRightAnalogInput = 3;

    // REV Pneumatic Hub solenoid addresses

    // Digital IO addresses

    // Operator input USB ports
    public static final int kLeftJoystick = 0;
    public static final int kRightJoystick = 1;
    public static final int kButtonBox = 2;

    // Driver Buttons
    public static final int kLeftLockWheels = 4;
    public static final int kRightResetNavXButton = 10;

    // Co-Piolt Button board

    // Zeroed values, should be in radians
    // source is google document in Electrical for team - module data
    public static final double kDegreesToRadians = Math.PI * 2.0 / 360.0;
    public static final double kZeroedFrontLeft = 178.8 * kDegreesToRadians;    // for FL module 4
    public static final double kZeroedFrontRight = 3.7 * kDegreesToRadians;   // for FR module 3
    public static final double kZeroedBackLeft = 138.9 * kDegreesToRadians;     // for BL module 1
    public static final double kZeroedBackRight = 212.5 * kDegreesToRadians;    // for BR module 2
};