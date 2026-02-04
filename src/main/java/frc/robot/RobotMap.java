// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotMap {

    // Swerve Motor Assemblies - SparkMax
    // Drive motor CAN == module #
    // Steer motor CAN == module # + 10
    public static final int kFrontLeftModule = 2;
    public static final int kBackLeftModule = 4;
    public static final int kBackRightModule = 1;
    public static final int kFrontRightModule = 3;

    // CAN motor addresses
        // SparkMax
        // Kraken
    public static final int kTopShooterWheelkraken = 21;
    public static final int kBottomShooterWheelkraken = 22;

    // Analog Input addresses
    public static final int kFrontLeftAnalogInput = 2; 
    public static final int kBackLeftAnalogInput = 0;    // swerve module 4
    public static final int kBackRightAnalogInput = 1;
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
    public static final int kfire = 1;

    // Zeroed values, should be in radians
    // source is google document in Electrical for team - module data
    public static final double kDegreesToRadians = Math.PI * 2.0 / 360.0;
    public static final double kZeroedFrontLeft = 32.1  * kDegreesToRadians;    // for FL module 2
    public static final double kZeroedFrontRight = 3.7 * kDegreesToRadians;   // for FR module 3
    public static final double kZeroedBackLeft = 89.1 * kDegreesToRadians;     // for BL module 4
    public static final double kZeroedBackRight = 50.4 * kDegreesToRadians;    // for BR module 1
};