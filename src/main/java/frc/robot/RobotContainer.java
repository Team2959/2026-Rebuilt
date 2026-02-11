// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ShooterVelocityfromDistanceCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.robotarians.Conditioning;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  // private final ShooterSubsytem m_ShooterSubsytem = new ShooterSubsytem();
  // private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  // private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
  // private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

  private final Conditioning m_driveXConditioning = new Conditioning();
  private final Conditioning m_driveYConditioning = new Conditioning();
  private final Conditioning m_turnConditioning = new Conditioning();
  private static double m_speedMultiplier = 1.0;

  private final CommandJoystick m_leftJoystick = new CommandJoystick(RobotMap.kLeftJoystick);
  private final CommandJoystick m_rightJoystick = new CommandJoystick(RobotMap.kRightJoystick);
  private final CommandJoystick m_buttonBox = new CommandJoystick(RobotMap.kButtonBox); 

  private final Robot m_robot;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    m_robot = robot;

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(m_driveSubsystem,
      () -> getDriveXInput(), () -> getDriveYInput(), () -> getTurnInput(),
      () -> m_robot.isTeleopEnabled()));

    m_rightJoystick.button(RobotMap.kRightResetNavXButton).onTrue(new InstantCommand(() -> {m_driveSubsystem.resetNavX();}, m_driveSubsystem));
    m_leftJoystick.button(RobotMap.kLeftLockWheels).whileTrue(m_driveSubsystem.lockWheelsCommand());

    // m_buttonBox.button(RobotMap.kIntake).toggleOnTrue(m_intakeSubsystem.toggleIntakeCommand());
    // m_buttonBox.button(RobotMap.kReverseIntake).whileTrue(m_intakeSubsystem.reverseIntakeCommand());
    
    // m_buttonBox.button(RobotMap.kfire).whileTrue(new ShooterVelocityfromDistanceCommand(m_ShooterSubsytem));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public double getDriveXInput()
  {
    // We getY() here because of the FRC coordinate system being turned 90 degrees
    return m_driveXConditioning.condition(-m_leftJoystick.getY())
            * DriveSubsystem.kMaxSpeedMetersPerSecond
            * m_speedMultiplier;
  }

  public double getDriveYInput()
  {
    // We getX() here becasuse of the FRC coordinate system being turned 90 degrees
    return m_driveYConditioning.condition(-m_leftJoystick.getX())
            * DriveSubsystem.kMaxSpeedMetersPerSecond
            * m_speedMultiplier;
  }

  public double getTurnInput()
  {
    return m_turnConditioning.condition(-m_rightJoystick.getX())
            * DriveSubsystem.kMaxAngularSpeedRadiansPerSecond
            * m_speedMultiplier;
  }

  public void initialize()
  {
    // m_speedMultiplier = m_speedSub.get();

    m_driveSubsystem.initialize();
  }
}
