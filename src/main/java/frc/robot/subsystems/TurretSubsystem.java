// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class TurretSubsystem extends SubsystemBase {

  private SparkMax m_turretMotor = new SparkMax(RobotMap.kTurretMotorSparkMax, MotorType.kBrushless);
  private SparkRelativeEncoder m_turretEncoder;
  private SparkClosedLoopController m_turretController;
  private final SparkMaxConfig m_turretConfig;

  private static final double kP = 0.0005;
  private static final double kI = 0.000001;
  private static final double kD = 0.0;
  private static final double KFf = 0.0;

  private final DoubleSubscriber m_kPSub;
  private final DoubleSubscriber m_kISub;
  private final DoubleSubscriber m_kDSub;
  private final DoubleSubscriber m_kFfSub;
  private final DoubleSubscriber m_targetPositionSub;

  private final BooleanSubscriber m_goToTargetSub;
  private final BooleanPublisher m_goToTargetPub;
  private final BooleanSubscriber m_updatePidSub;
  private final BooleanPublisher m_updatePidPub;

  private final DoublePublisher m_motorAppliedOutputPub;
  private final DoublePublisher m_velocityPub;
  private final DoublePublisher m_positionPub;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    m_turretEncoder = (SparkRelativeEncoder) m_turretMotor.getEncoder();
    m_turretController = m_turretMotor.getClosedLoopController();
    m_turretConfig = new SparkMaxConfig();
    m_turretConfig.idleMode(IdleMode.kBrake);

    m_turretConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD);

    m_turretMotor.configure(m_turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get the subtable called "datatable"
    NetworkTable datatable = inst.getTable("Turret");
    // subscribe to the topic in "datatable" called "target angle"
    // default value is 0
    var targetDoubleTopic = datatable.getDoubleTopic("target angle");
    targetDoubleTopic.publish().set(0);
    m_targetPositionSub = targetDoubleTopic.subscribe(0.0);

    var kpTopic = datatable.getDoubleTopic("Turret Kp");
    kpTopic.publish().set(kP);
    m_kPSub = kpTopic.subscribe(kP);

    var kiTopic = datatable.getDoubleTopic("Turret Ki");
    kiTopic.publish().set(kI);
    m_kISub = kiTopic.subscribe(kI);

    var dTopic = datatable.getDoubleTopic("Turret D");
    dTopic.publish().set(kD);
    m_kDSub = dTopic.subscribe(kD);

    var ffTopic = datatable.getDoubleTopic("Turret FF");
    ffTopic.publish().set(KFf);
    m_kFfSub = ffTopic.subscribe(KFf);

    var updatePIDTopic = datatable.getBooleanTopic("update Turret PID");
    m_updatePidPub = updatePIDTopic.publish();
    m_updatePidPub.set(false);
    m_updatePidSub = updatePIDTopic.subscribe(false);

    var goToTargetAngle = datatable.getBooleanTopic("go To Target Angle");
    m_goToTargetPub = goToTargetAngle.publish();
    m_goToTargetPub.set(false);
    m_goToTargetSub = goToTargetAngle.subscribe(false);

    m_positionPub = datatable.getDoubleTopic("Position").publish();
    m_positionPub.set(0);

    m_velocityPub = datatable.getDoubleTopic("Velocity").publish();
    m_velocityPub.set(0);

    m_motorAppliedOutputPub = datatable.getDoubleTopic("MotorAppliedOutput").publish();
    m_motorAppliedOutputPub.set(0);
  }

  int m_ticks = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_ticks++;
    if (m_ticks % 15 != 5)
      return;

    m_positionPub.set(m_turretEncoder.getPosition());
    m_velocityPub.set(m_turretEncoder.getVelocity());
    m_motorAppliedOutputPub.set(m_turretMotor.getAppliedOutput());

    dashboardUpdate();
  }

  public void stopTurret() {
    m_turretMotor.set(0);
  }

  public void goToTargetAngle(double targetAngle) {
    // currently using rotations
    // ToDo: limit input to min/max allowed angles, if out of range, don't change the target
    // ToDo: change to using degrees as the input, have a conversion from degrees to position
    m_turretController.setSetpoint(targetAngle, ControlType.kPosition);
  }

  public void dashboardUpdate() {
    if (m_updatePidSub.get()) {
      double kp = m_kPSub.get();
      double ki = m_kISub.get();
      double kd = m_kDSub.get();
      double kff = m_kFfSub.get();

      m_turretConfig.closedLoop.pid(kp, ki, kd);
      m_turretMotor.configure(m_turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      m_updatePidPub.set(false);
    }

    if (m_goToTargetSub.get()) {
      goToTargetAngle(m_targetPositionSub.get());

      m_goToTargetPub.set(false);
    }
  }
}
