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
import com.revrobotics.spark.ClosedLoopSlot;
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
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.robotarians.NeoPidNetworkTableHelper;
import frc.robot.robotarians.PidValuesRecord;

public class IntakeSubsystem extends SubsystemBase {
  public enum ExtendIntakePositionType {
    Retracted,
    Extended,
    JostleUp
  }

  private SparkMax m_intakeMotor = new SparkMax(RobotMap.kIntakeMotorSparkMax, MotorType.kBrushless);
  private SparkMax m_extendIntakeMotor = new SparkMax(RobotMap.KIntakeExtendMotorSparkMax, MotorType.kBrushless);
  private SparkMaxConfig m_extendConfig;
  private SparkRelativeEncoder m_extendEncoder;
  private SparkClosedLoopController m_extendController;

  private static final double defaultSpeed = 1;
  private final DoubleSubscriber m_IntakeSpeedSub;
  private double m_IntakeSpeed = defaultSpeed;  // power and current limiting
  private static final int kExtendCurrentLimitAmps = 30;
  // following Rev's arm kS and kG voltage measurements for feed forward
  // https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control
  // V1 = 0.35; V2 = 0.12
  // kS = (V1 - V2)/2.0
  // kCosG = V2 + kS
  private static final double kStatic = 0.115;
  private static final double kCosG = 0.235;
  private static final double kCosRatio = 3 * 29.97; // motor 3:1 * 9:1 * gears = 29.97
  // kP was 0.15 in initial testing
  private static final PidValuesRecord retactPidValues = new PidValuesRecord(0.07, 0.0, 0);
  // kSlot1 for extending with more power, separate tuning
  private static final PidValuesRecord extendPidValues = new PidValuesRecord(0.07, 0.0, 0);
  private static final double kExtendMaxOutput = 1.0;

  private final NeoPidNetworkTableHelper m_networkTable = new NeoPidNetworkTableHelper("Intake Extend",
      retactPidValues);
  private final IntegerSubscriber m_currentLimitSub;
  private final DoubleSubscriber m_maxOutputSub;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    var intakeConfig = new SparkMaxConfig();
    intakeConfig.inverted(true).idleMode(IdleMode.kCoast);
    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_extendEncoder = (SparkRelativeEncoder) m_extendIntakeMotor.getEncoder();
    m_extendController = m_extendIntakeMotor.getClosedLoopController();

    m_extendConfig = new SparkMaxConfig();
    m_extendConfig.idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(kExtendCurrentLimitAmps)
        .voltageCompensation(12.6);

    m_extendConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .allowedClosedLoopError(0.1, ClosedLoopSlot.kSlot0)
        .allowedClosedLoopError(0.1, ClosedLoopSlot.kSlot1)
        .pid(retactPidValues.kP(), retactPidValues.kI(), retactPidValues.kD())
        .pid(extendPidValues.kP(), extendPidValues.kI(), extendPidValues.kD(), ClosedLoopSlot.kSlot1)
        .outputRange(-kExtendMaxOutput, kExtendMaxOutput);
    m_extendConfig.closedLoop.feedForward.kS(kStatic).kCos(kCosG).kCosRatio(kCosRatio);
    m_extendIntakeMotor.configure(m_extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("Intake");

    var speedTopic = datatable.getDoubleTopic("IntakeSpeed");
    speedTopic.publish().set(defaultSpeed);
    m_IntakeSpeedSub = speedTopic.subscribe(defaultSpeed);

    var currentLimitTopic = m_networkTable.networkTable().getIntegerTopic("Current Limit");
    currentLimitTopic.publish().set(kExtendCurrentLimitAmps);
    m_currentLimitSub = currentLimitTopic.subscribe(kExtendCurrentLimitAmps);

    var maxOutputTopic = m_networkTable.networkTable().getDoubleTopic("Max Output");
    maxOutputTopic.publish().set(kExtendMaxOutput);
    m_maxOutputSub = maxOutputTopic.subscribe(kExtendMaxOutput);

    setExtendPosition(0, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (RobotContainer.m_ticks % 15 != 3)
      return;

    dashboardUpdate();
  }

  private void dashboardUpdate() {
    m_IntakeSpeed = m_IntakeSpeedSub.get();

    m_networkTable.dashboardUpdate(m_extendIntakeMotor, m_extendEncoder,
        m_extendConfig, (t) -> setExtendPosition(t, ClosedLoopSlot.kSlot0),
        (b) -> moreMotorUpdates());
  }

  private void startIntake() {
    m_intakeMotor.set(m_IntakeSpeed);
  }

  private void stopIntake() {
    m_intakeMotor.set(0);
  }

  private void reverseIntake() {
    m_intakeMotor.set(-m_IntakeSpeed * 0.5);
  }

  public Command reverseIntakeCommand() {
    return new StartEndCommand(() -> reverseIntake(), () -> stopIntake(), this);
  }

  public Command toggleIntakeCommand() {
    return new StartEndCommand(() -> startIntake(), () -> stopIntake(), this);
  }

  private void setExtendPosition(double position, ClosedLoopSlot slot) {
    // currnently in units of rotations
    m_extendController.setSetpoint(position, ControlType.kPosition, slot);
  }

  public void setExtendPosition(ExtendIntakePositionType targetPosition) {
    var position = positionFromPositionType(targetPosition);
    var slot = ClosedLoopSlot.kSlot0;
    if (targetPosition == ExtendIntakePositionType.Extended) {
      slot = ClosedLoopSlot.kSlot1;
    }
    setExtendPosition(position, slot);
  }

  public Command extendIntakeCommand() {
    return new InstantCommand(() -> {
      setExtendPosition(ExtendIntakePositionType.Extended);
      startIntake();
    }, this);
  }

  public Command jostleUpIntakeCommand() {
    return new InstantCommand(() -> {
      setExtendPosition(ExtendIntakePositionType.JostleUp);
    }, this);
  }

  public Command retractIntakeCommand() {
    return new InstantCommand(() -> {
      setExtendPosition(ExtendIntakePositionType.Retracted);
      stopIntake();
    }, this);
  }

  private void moreMotorUpdates() {
    m_extendConfig.smartCurrentLimit((int) m_currentLimitSub.get());
    var maxOutput = m_maxOutputSub.get();
    m_extendConfig.closedLoop.outputRange(-maxOutput, maxOutput);
  }

  public boolean isAtPosition(ExtendIntakePositionType position) {
    return Math.abs(m_extendEncoder.getPosition() - positionFromPositionType(position)) < 1;
  }

  private double positionFromPositionType(ExtendIntakePositionType position) {
    switch (position) {
      case Extended:
        return 37.5;
      case JostleUp:
        return 20;
      default:
        return 0;
    }
  }
}
