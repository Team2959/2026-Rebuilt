// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.robotarians.NeoPidNetworkTableHelper;
import frc.robot.robotarians.PidValuesRecord;

public class IntakeSubsystem extends SubsystemBase {
  public enum ExtendIntakePositionType {
    Retracted,
    Extended
  }

  private SparkMax m_intakeMotor = new SparkMax(RobotMap.kIntakeMotorSparkMax, MotorType.kBrushless);
  private SparkMax m_extendIntakeMotor = new SparkMax(RobotMap.KIntakeExtendMotorSparkMax, MotorType.kBrushless);
  private SparkMaxConfig m_extendConfig;
  private SparkRelativeEncoder m_extendEncoder;
  private SparkClosedLoopController m_extendController;

  private static final double defaultSpeed = 1.0;
  private final DoubleSubscriber m_IntakeSpeedSub;
  private double m_IntakeSpeed = defaultSpeed;

  private static final double defaultReverseSpeed = -.5;
  private final DoubleSubscriber m_ReverseIntakeSpeedSub;
  private double m_reverseIntakeSpeed = defaultReverseSpeed;

  private static final int defaultExtendedPosition = 12;

  private static final int kExtendCurrentLimitAmps = 20;
  private static final double kExtendMaxOutput = 0.5;
  private static final double kStatic = 0.15;
  private static final double kCosG = 0.3;
  private static final double kCosRatio = 29.97; // motor 9:1 * gears = 29.97
  private static final PidValuesRecord pidValues = new PidValuesRecord(0.15, 0.0, 0);

  private final NeoPidNetworkTableHelper m_networkTable = new NeoPidNetworkTableHelper("Intake Extend", pidValues);
  private final IntegerSubscriber m_currentLimitSub;
  private final DoubleSubscriber m_maxOutputSub;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    var intakeConfig = new SparkMaxConfig();
    intakeConfig.inverted(true);
    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_extendEncoder = (SparkRelativeEncoder) m_extendIntakeMotor.getEncoder();
    m_extendController = m_extendIntakeMotor.getClosedLoopController();

    m_extendConfig = new SparkMaxConfig();
    // ToDo: switch back to brake mode
    m_extendConfig.idleMode(IdleMode.kCoast)
        .inverted(false)
        .smartCurrentLimit(kExtendCurrentLimitAmps)
        .voltageCompensation(12.6);

    m_extendConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(pidValues.kP(), pidValues.kI(), pidValues.kD())
        .outputRange(-kExtendMaxOutput, kExtendMaxOutput);
    m_extendConfig.closedLoop.feedForward.kS(kStatic).kCos(kCosG).kCosRatio(kCosRatio);
    m_extendIntakeMotor.configure(m_extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("Intake");

    var speedTopic = datatable.getDoubleTopic("IntakeSpeed");
    speedTopic.publish().set(defaultSpeed);
    m_IntakeSpeedSub = speedTopic.subscribe(defaultSpeed);

    var reverseSpeedTopic = datatable.getDoubleTopic("ReverseIntakeSpeed");
    reverseSpeedTopic.publish().set(defaultReverseSpeed);
    m_ReverseIntakeSpeedSub = reverseSpeedTopic.subscribe(defaultReverseSpeed);

    var currentLimitTopic = m_networkTable.networkTable().getIntegerTopic("Current Limit");
    currentLimitTopic.publish().set(kExtendCurrentLimitAmps);
    m_currentLimitSub = currentLimitTopic.subscribe(kExtendCurrentLimitAmps);

    var maxOutputTopic = m_networkTable.networkTable().getDoubleTopic("Max Output");
    maxOutputTopic.publish().set(kExtendMaxOutput);
    m_maxOutputSub = maxOutputTopic.subscribe(kExtendMaxOutput);
  }

  int m_ticks = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_ticks++;
    if (m_ticks % 15 != 3)
      return;

    dashboardUpdate();
  }

  private void startIntake() {
    m_intakeMotor.set(m_IntakeSpeed);
  }

  private void stopIntake() {
    m_intakeMotor.set(0);
  }

  private void reverseIntake() {
    m_intakeMotor.set(m_reverseIntakeSpeed);
  }

  public Command reverseIntakeCommand() {
    return new StartEndCommand(() -> reverseIntake(), () -> stopIntake(), this);
  }

  public Command toggleIntakeCommand() {
    return new StartEndCommand(() -> startIntake(), () -> stopIntake(), this);
  }

  private void setExtendPosition(double position) {
    // currnently in units of rotations
    // ToDo: may need to limit motor power
    m_extendController.setSetpoint(position, ControlType.kPosition);
  }

  private void setExtendPosition(ExtendIntakePositionType target) {
    var newPosition = 0;
    if (target == ExtendIntakePositionType.Extended) {
      newPosition = defaultExtendedPosition;
    }
    setExtendPosition(newPosition);
  }

  public Command extendIntakeCommand() {
    return new InstantCommand(() -> setExtendPosition(ExtendIntakePositionType.Extended), this);
  }

  public Command retractIntakeCommand() {
    return new InstantCommand(() -> setExtendPosition(ExtendIntakePositionType.Retracted), this);
  }

  public void dashboardUpdate() {
    m_IntakeSpeed = m_IntakeSpeedSub.get();
    m_reverseIntakeSpeed = m_ReverseIntakeSpeedSub.get();

    m_networkTable.dashboardUpdate(m_extendIntakeMotor, m_extendEncoder,
        m_extendConfig, (t) -> setExtendPosition(t),
        (b) -> moreMotorUpdates());
  }

  private void moreMotorUpdates() {
    m_extendConfig.smartCurrentLimit((int) m_currentLimitSub.get());
    var maxOutput = m_maxOutputSub.get();
    m_extendConfig.closedLoop.outputRange(-maxOutput, maxOutput);
  }
}
