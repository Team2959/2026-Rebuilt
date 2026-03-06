// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoFeedShooterCommand;
import frc.robot.commands.ShooterVelocityfromDistanceCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.commands.TurretAutoTargetCommand;
import frc.robot.commands.TurretToAngleCommand;
import frc.robot.robotarians.Conditioning;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsytem;
import frc.robot.subsystems.TurretSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final HopperSubsystem m_hopperSubsystem = new HopperSubsystem();
  private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
  private final ShooterSubsytem m_ShooterSubsytem = new ShooterSubsytem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

  private final Conditioning m_driveXConditioning = new Conditioning();
  private final Conditioning m_driveYConditioning = new Conditioning();
  private final Conditioning m_turnConditioning = new Conditioning();
  private static double m_speedMultiplier = 1.0;

  private final CommandJoystick m_leftJoystick = new CommandJoystick(RobotMap.kLeftJoystick);
  private final CommandJoystick m_rightJoystick = new CommandJoystick(RobotMap.kRightJoystick);
  private final CommandJoystick m_buttonBox = new CommandJoystick(RobotMap.kButtonBox);

  private final Robot m_robot;

  private final SendableChooser<Command> m_autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {
    m_robot = robot;

    // path planner auto set up
    createNamedCommandsForAutos();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
        () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(),
        () -> m_robot.isTeleopEnabled()));

    m_turretSubsystem.setDefaultCommand(new TurretAutoTargetCommand(m_turretSubsystem,
        () -> {
          return m_driveSubsystem.getAngle().getDegrees();
        }));

    m_rightJoystick.button(RobotMap.kRightResetNavXButton).onTrue(
        new InstantCommand(() -> {
          m_driveSubsystem.resetNavX();
        }, m_driveSubsystem));
    m_leftJoystick.button(RobotMap.kLeftLockWheels).whileTrue(m_driveSubsystem.lockWheelsCommand());

    m_buttonBox.button(RobotMap.kExtendIntake).onTrue(extendIntakeCommand());
    m_buttonBox.button(RobotMap.kRetractIntake).onTrue(m_intakeSubsystem.retractIntakeCommand());
    m_buttonBox.button(RobotMap.kToggleIntake).toggleOnTrue(m_intakeSubsystem.toggleIntakeCommand());
    m_buttonBox.button(RobotMap.kReverseIntake).whileTrue(m_intakeSubsystem.reverseIntakeCommand());
    m_buttonBox.button(RobotMap.kToggleHopper).toggleOnTrue(m_hopperSubsystem.toggleHopperCommand());
    m_buttonBox.button(RobotMap.kReverseHopper).whileTrue(m_hopperSubsystem.reverseHopperCommand());
    m_buttonBox.button(RobotMap.kFeedShooter).whileTrue(
        new StartEndCommand(() -> m_FeederSubsystem.startFeeder(), () -> m_FeederSubsystem.stopFeeder(),
            m_FeederSubsystem));
    m_buttonBox.button(RobotMap.kReverseFeeder).whileTrue(m_FeederSubsystem.reverseFeederCommand());
    m_buttonBox.button(RobotMap.kSuspendAutoTurret).toggleOnTrue(new StartEndCommand(
        () -> m_turretSubsystem.setSuspendAutoTurret(true), () -> m_turretSubsystem.setSuspendAutoTurret(false)));

    m_buttonBox.button(RobotMap.kFire).onTrue(startShootingCommand());
    m_buttonBox.button(RobotMap.kStopFire).onTrue(stopShootingCommand());
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

  private Command startShootingCommand() {
    return new ShooterVelocityfromDistanceCommand(m_ShooterSubsytem)
        .alongWith(m_hopperSubsystem.startHopperCommand()
            .andThen(new AutoFeedShooterCommand(m_FeederSubsystem, m_ShooterSubsytem, m_turretSubsystem)));
  }

  private Command stopShootingCommand() {
    return m_ShooterSubsytem.shooterToIdleCommand()
        .andThen(new InstantCommand(() -> {
          if (m_intakeSubsystem.isRetracted())
            m_hopperSubsystem.stopHopper();
        }));
  }

  private Command extendIntakeCommand() {
    return m_intakeSubsystem.extendIntakeCommand()
        .andThen(m_hopperSubsystem.startHopperCommand());
  }

  private Command startAndStopShooting(double durationInSeconds) {
    return startShootingCommand()
        .alongWith(new WaitCommand(durationInSeconds).andThen(stopShootingCommand()));
  }

  private void createNamedCommandsForAutos() {
    NamedCommands.registerCommand("Shoot First 8", startAndStopShooting(2));
    NamedCommands.registerCommand("Shoot Full Hopper", startAndStopShooting(5));
    NamedCommands.registerCommand("Start Shooting", startShootingCommand());
    NamedCommands.registerCommand("Stop Shooting", stopShootingCommand());
    NamedCommands.registerCommand("Extend Intake", extendIntakeCommand());
    NamedCommands.registerCommand("Turret to Pos 60", new TurretToAngleCommand(m_turretSubsystem, 60));
    NamedCommands.registerCommand("Turret to Neg 60", new TurretToAngleCommand(m_turretSubsystem, -60));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
