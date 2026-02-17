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

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

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

  private static final int defaultExtendedPosition = 50;

  private static final int kExtendCurrentLimitAmps = 40;
  private static final double extendKp = 0.01;
  private static final double extendKi = 0.0;
  private static final double extendKd = 0.0;

  private final DoubleSubscriber m_kPSub;
  private final DoubleSubscriber m_kISub;
  private final DoubleSubscriber m_kDSub;
  private final DoubleSubscriber m_extendTargetSub;
  private final DoublePublisher m_extendPositionPub;

  private final BooleanSubscriber m_goToTargetSub;
  private final BooleanPublisher m_goToTargetPub;
  private final BooleanSubscriber m_updatePidSub;
  private final BooleanPublisher m_updatePidPub;

  private final DoublePublisher m_extendAppliedOutputPub;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_extendEncoder = (SparkRelativeEncoder) m_extendIntakeMotor.getEncoder();
    m_extendController = m_extendIntakeMotor.getClosedLoopController();

    m_extendConfig = new SparkMaxConfig();
    m_extendConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kExtendCurrentLimitAmps)
        .voltageCompensation(12.6);

    m_extendConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(extendKp, extendKi, extendKd);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("Intake");

    var speedTopic = datatable.getDoubleTopic("IntakeSpeed");
    speedTopic.publish().set(defaultSpeed);
    m_IntakeSpeedSub = speedTopic.subscribe(defaultSpeed);

    var reverseSpeedTopic = datatable.getDoubleTopic("ReverseIntakeSpeed");
    reverseSpeedTopic.publish().set(defaultSpeed);
    m_ReverseIntakeSpeedSub = reverseSpeedTopic.subscribe(defaultSpeed);

    // PID topic
    var kpTopic = datatable.getDoubleTopic("kP");
    kpTopic.publish().set(extendKp);
    m_kPSub = kpTopic.subscribe(extendKp);
    var kiTopic = datatable.getDoubleTopic("kI");
    kiTopic.publish().set(extendKi);
    m_kISub = kiTopic.subscribe(extendKi);
    var kDTopic = datatable.getDoubleTopic("kD");
    kDTopic.publish().set(extendKd);
    m_kDSub = kDTopic.subscribe(extendKd);

    var ExtendTargetTopic = datatable.getDoubleTopic("Extend Target");
    ExtendTargetTopic.publish().set(0);
    m_extendTargetSub = ExtendTargetTopic.subscribe(0);

    m_extendPositionPub = datatable.getDoubleTopic("Extend Position").publish();

    var goToTarget = datatable.getBooleanTopic("go to Target");
    m_goToTargetPub = goToTarget.publish();
    m_goToTargetPub.set(false);
    m_goToTargetSub = goToTarget.subscribe(false);

    var updatePID = datatable.getBooleanTopic("update PID");
    m_updatePidPub = updatePID.publish();
    m_updatePidPub.set(false);
    m_updatePidSub = updatePID.subscribe(false);

    m_extendAppliedOutputPub = datatable.getDoubleTopic("Extend Applied Output").publish();
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

  private void setExtendPosition(double position){
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

    m_extendPositionPub.set(m_extendEncoder.getPosition());
    m_extendAppliedOutputPub.set(m_extendIntakeMotor.getAppliedOutput());

    if (m_updatePidSub.get()) {
      m_extendConfig.closedLoop.pid(m_kPSub.get(), m_kISub.get(), m_kDSub.get());
      m_extendIntakeMotor.configure(m_extendConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      m_updatePidPub.set(false);
    }

    if (m_goToTargetSub.get()) {
      setExtendPosition(m_extendTargetSub.get());

      m_goToTargetPub.set(false);
    }
  }
}
