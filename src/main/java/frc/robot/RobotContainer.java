// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ShooterVelocityfromDistanceCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.commands.TurretAutoTargetCommand;
import frc.robot.robotarians.Conditioning;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsytem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsytem.ShooterStateType;
import frc.robot.subsystems.ClimbExtendSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final HopperSubsystem m_hopperSubsystem = new HopperSubsystem();
  private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
  private final ShooterSubsytem m_ShooterSubsytem = new ShooterSubsytem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  // private final ClimbExtendSubsystem m_climbExtendSubsystem = new
  // ClimbExtendSubsystem();

  private final Conditioning m_driveXConditioning = new Conditioning();
  private final Conditioning m_driveYConditioning = new Conditioning();
  private final Conditioning m_turnConditioning = new Conditioning();
  private static double m_speedMultiplier = 1.0;

  private final CommandJoystick m_leftJoystick = new CommandJoystick(RobotMap.kLeftJoystick);
  private final CommandJoystick m_rightJoystick = new CommandJoystick(RobotMap.kRightJoystick);
  private final CommandJoystick m_buttonBox = new CommandJoystick(RobotMap.kButtonBox);

  private final Robot m_robot;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {
    m_robot = robot;

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
        () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(),
        () -> m_robot.isTeleopEnabled()));

    // m_turretSubsystem.setDefaultCommand(new TurretAutoTarget(m_turretSubsystem,
    //     () -> {
    //       return m_driveSubsystem.getAngle().getDegrees();
    //     }));

    m_rightJoystick.button(RobotMap.kRightResetNavXButton).onTrue(
        new InstantCommand(() -> {
          m_driveSubsystem.resetNavX();
        }, m_driveSubsystem));
    m_leftJoystick.button(RobotMap.kLeftLockWheels).whileTrue(m_driveSubsystem.lockWheelsCommand());

    m_buttonBox.button(RobotMap.kExtendIntake)
      .onTrue(m_intakeSubsystem.extendIntakeCommand()
        .andThen(m_hopperSubsystem.startHopperCommand()));
    m_buttonBox.button(RobotMap.kRetractIntake).onTrue(m_intakeSubsystem.retractIntakeCommand());
    m_buttonBox.button(RobotMap.kToggleIntake).toggleOnTrue(m_intakeSubsystem.toggleIntakeCommand());
    m_buttonBox.button(RobotMap.kReverseIntake).whileTrue(m_intakeSubsystem.reverseIntakeCommand());
    m_buttonBox.button(RobotMap.kToggleHopper).toggleOnTrue(m_hopperSubsystem.toggleHopperCommand());
    m_buttonBox.button(RobotMap.kReverseHopper).whileTrue(m_hopperSubsystem.reverseHopperCommand());

    m_buttonBox.button(RobotMap.kFire).onTrue(
      new ShooterVelocityfromDistanceCommand(m_ShooterSubsytem)
      .alongWith(m_hopperSubsystem.startHopperCommand()));
    m_buttonBox.button(RobotMap.kStopFire).onTrue(
      m_FeederSubsystem.stopfeederCommand()
      .alongWith(m_ShooterSubsytem.shooterToIdleCommand()));

    Trigger startFeederTrigger = new Trigger(() -> m_ShooterSubsytem.getShooterState() == ShooterStateType.Shooting);
    startFeederTrigger.onTrue(m_FeederSubsystem.startfeederCommand());
    Trigger atVelocityTrigger = new Trigger(() -> m_ShooterSubsytem.isAtVelocity());
    Trigger atAngleTrigger = new Trigger(() -> m_turretSubsystem.isAtAngle());
    atVelocityTrigger.and(atAngleTrigger).onTrue(
      new InstantCommand(() -> m_ShooterSubsytem.setShooterState(ShooterStateType.Shooting)));
  }

  public double getDriveXInput() {
    // We getY() here because of the FRC coordinate system being turned 90 degrees
    return m_driveXConditioning.condition(-m_leftJoystick.getY())
        * DriveSubsystem.kMaxSpeedMetersPerSecond
        * m_speedMultiplier;
  }

  public double getDriveYInput() {
    // We getX() here becasuse of the FRC coordinate system being turned 90 degrees
    return m_driveYConditioning.condition(-m_leftJoystick.getX())
        * DriveSubsystem.kMaxSpeedMetersPerSecond
        * m_speedMultiplier;
  }

  public double getTurnInput() {
    return m_turnConditioning.condition(-m_rightJoystick.getX())
        * DriveSubsystem.kMaxAngularSpeedRadiansPerSecond
        * m_speedMultiplier;
  }

  public void initialize() {
    // m_speedMultiplier = m_speedSub.get();

    m_driveSubsystem.initialize();
  }
}
