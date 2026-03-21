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
import frc.robot.vision.AprilTagShooterHelpers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private double m_speedMultiplier = 0.85;
  private double m_shootingSpeedReduction = 1.0;

  private final CommandJoystick m_leftJoystick = new CommandJoystick(RobotMap.kLeftJoystick);
  private final CommandJoystick m_rightJoystick = new CommandJoystick(RobotMap.kRightJoystick);
  private final CommandJoystick m_buttonBox = new CommandJoystick(RobotMap.kButtonBox);

  private final Robot m_robot;
  private double m_targetTurretAngle;
  private double m_targetDistance;

  private final SendableChooser<Command> m_autoChooser;
  private final DoubleSubscriber m_speedSub;
  private final DoublePublisher m_mt2TargetAnglePub;
  private final DoublePublisher m_mt2TargetDistancePub;
  private final BooleanPublisher m_atDistancePub;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {
    m_robot = robot;

    // path planner auto set up
    createNamedCommandsForAutos();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    var datatable = inst.getTable("Speed Multiplier");
    var topic = datatable.getDoubleTopic("Speed Multiplier");
    topic.publish().set(m_speedMultiplier);
    m_speedSub = topic.subscribe(m_speedMultiplier);
    topic = datatable.getDoubleTopic("MT2 Distance");
    m_mt2TargetDistancePub = topic.publish();
    topic = datatable.getDoubleTopic("MT2 Angle");
    m_mt2TargetAnglePub = topic.publish();
    var topic2 = datatable.getBooleanTopic("Is At Distance");
    m_atDistancePub = topic2.publish();

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
        () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(),
        () -> m_robot.isTeleopEnabled()));

    m_turretSubsystem.setDefaultCommand(new TurretAutoTargetCommand(m_turretSubsystem,
        () -> {
          return m_targetTurretAngle;
        },
        () -> {
          return m_driveSubsystem.getYawRate();
        }));

    m_rightJoystick.button(RobotMap.kRightResetNavXButton).onTrue(
        new InstantCommand(() -> {
          m_driveSubsystem.resetNavX();
        }, m_driveSubsystem));
    m_leftJoystick.button(RobotMap.kLeftLockWheels).whileTrue(m_driveSubsystem.lockWheelsCommand());
    m_leftJoystick.button(RobotMap.kLeftFixedShooterButton).toggleOnTrue(new StartEndCommand(
        () -> m_ShooterSubsytem.setFixedShooterSpeed(true),
        () -> m_ShooterSubsytem.setFixedShooterSpeed(false)));

    // m_rightJoystick.button(1).whileTrue(m_turretSubsystem.sysIdQuasistatic(Direction.kForward));
    // m_leftJoystick.button(1).whileTrue(m_turretSubsystem.sysIdQuasistatic(Direction.kReverse));
    // m_rightJoystick.button(2).whileTrue(m_turretSubsystem.sysIdDynamic(Direction.kForward));
    // m_leftJoystick.button(2).whileTrue(m_turretSubsystem.sysIdDynamic(Direction.kReverse));

    m_buttonBox.button(RobotMap.kExtendIntake).onTrue(extendIntakeCommand());
    m_buttonBox.button(RobotMap.kRetractIntake).onTrue(m_intakeSubsystem.retractIntakeCommand());
    m_buttonBox.button(RobotMap.kIntakeJostleUp).onTrue(jostleUpIntakeCommand());
    m_buttonBox.button(RobotMap.kToggleIntake).toggleOnTrue(m_intakeSubsystem.toggleIntakeCommand());
    m_buttonBox.button(RobotMap.kReverseIntake).whileTrue(m_intakeSubsystem.reverseIntakeCommand());
    m_buttonBox.button(RobotMap.kToggleHopper).toggleOnTrue(m_hopperSubsystem.toggleHopperCommand());
    m_buttonBox.button(RobotMap.kReverseHopper).whileTrue(m_hopperSubsystem.reverseHopperCommand());
    m_buttonBox.button(RobotMap.kFeedShooter).whileTrue(
        new StartEndCommand(() -> m_FeederSubsystem.startFeeder(), () -> m_FeederSubsystem.stopFeeder(),
            m_FeederSubsystem));
    m_buttonBox.button(RobotMap.kReverseFeeder).whileTrue(m_FeederSubsystem.reverseFeederCommand());
    m_buttonBox.button(RobotMap.kSuspendAutoTurret).toggleOnTrue(new StartEndCommand(
        () -> m_turretSubsystem.setSuspendAutoTurret(true),
        () -> m_turretSubsystem.setSuspendAutoTurret(false)));

    m_buttonBox.button(RobotMap.kFire).onTrue(startShootingCommand());
    m_buttonBox.button(RobotMap.kStopFire).onTrue(stopShootingCommand());
  }

  public double getDriveXInput() {
    // We getY() here because of the FRC coordinate system being turned 90 degrees
    return m_driveXConditioning.condition(-m_leftJoystick.getY())
        * DriveSubsystem.kMaxSpeedMetersPerSecond
        * m_speedMultiplier * m_shootingSpeedReduction;
  }

  public double getDriveYInput() {
    // We getX() here becasuse of the FRC coordinate system being turned 90 degrees
    return m_driveYConditioning.condition(-m_leftJoystick.getX())
        * DriveSubsystem.kMaxSpeedMetersPerSecond
        * m_speedMultiplier * m_shootingSpeedReduction;
  }

  public double getTurnInput() {
    return m_turnConditioning.condition(-m_rightJoystick.getX())
        * DriveSubsystem.kMaxAngularSpeedRadiansPerSecond
        * m_speedMultiplier * m_shootingSpeedReduction;
  }

  public void initialize() {
    m_speedMultiplier = m_speedSub.get();

    m_driveSubsystem.initialize();
  }

  public static int m_ticks = 0;

  public void robotPeriodic() {
    // MegaTag2 targeting
    AprilTagShooterHelpers.updateLimelightPose(m_turretSubsystem.currentAngle());
    AprilTagShooterHelpers.updateRobotOrientation(m_driveSubsystem.getAngle().getDegrees(),
        m_driveSubsystem.getYawRate());
    var mt2 = AprilTagShooterHelpers.alliancePoseMt2();
    m_targetTurretAngle = AprilTagShooterHelpers.mt2TargetAngle(mt2, m_ShooterSubsytem.isShooting());
    m_targetDistance = AprilTagShooterHelpers.mt2DistanceToTaget(mt2, m_ShooterSubsytem.isShooting());

    m_atDistancePub.set(Math.abs(m_targetDistance - 2.0) < 0.2);

    m_ticks++;

    if (m_ticks % 15 != 1)
      return;

    m_mt2TargetAnglePub.set(m_targetTurretAngle);
    m_mt2TargetDistancePub.set(m_targetDistance);
  }

  public void autoInit() {
    // uncomment to force fixed shooter speed and/or turret angle in auto
    // m_ShooterSubsytem.setFixedShooterSpeed(true);
    // m_turretSubsystem.setSuspendAutoTurret(true);
  }

  public void teleOpInit() {
    // uncomment to force fixed shooter speed and/or turret angle in teleop
    // m_ShooterSubsytem.setFixedShooterSpeed(true);
    // m_turretSubsystem.setSuspendAutoTurret(true);
  }

  private Command startShootingCommand() {
    return new ShooterVelocityfromDistanceCommand(m_ShooterSubsytem, () -> {
      return m_targetDistance;
    })
        .alongWith(m_hopperSubsystem.startHopperCommand()
            .alongWith(new InstantCommand(() -> {
              m_shootingSpeedReduction = 0.50;
            })
                .andThen(new AutoFeedShooterCommand(m_FeederSubsystem, m_ShooterSubsytem, m_turretSubsystem))));
  }

  private Command stopShootingCommand() {
    return m_ShooterSubsytem.shooterToIdleCommand()
        .andThen(new InstantCommand(() -> {
          m_shootingSpeedReduction = 1.0;
          m_hopperSubsystem.stopHopper();
        }));
  }

  private Command extendIntakeCommand() {
    return m_intakeSubsystem.extendIntakeCommand();
  }

  private Command jostleUpIntakeCommand() {
    return m_intakeSubsystem.jostleUpIntakeCommand();
  }

  private Command startAndStopShooting(double durationInSeconds) {
    return startShootingCommand()
        .alongWith(new WaitCommand(durationInSeconds).andThen(stopShootingCommand()));
  }

  private void createNamedCommandsForAutos() {
    NamedCommands.registerCommand("Shoot First 8", startAndStopShooting(3));
    NamedCommands.registerCommand("Shoot Full Hopper", startAndStopShooting(10));
    NamedCommands.registerCommand("Start Shooting", startShootingCommand());
    NamedCommands.registerCommand("Stop Shooting", stopShootingCommand());
    NamedCommands.registerCommand("Extend Intake", extendIntakeCommand());
    NamedCommands.registerCommand("Jostle Up Intake", jostleUpIntakeCommand());
    NamedCommands.registerCommand("Turret to Pos 60", new TurretToAngleCommand(m_turretSubsystem, 60));
    NamedCommands.registerCommand("Turret to Neg 60", new TurretToAngleCommand(m_turretSubsystem, -60));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
