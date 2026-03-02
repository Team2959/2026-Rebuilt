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

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.robotarians.NeoPidNetworkTableHelper;
import frc.robot.robotarians.PidValuesRecord;
import frc.robot.vision.AprilTagShooterHelpers;

public class TurretSubsystem extends SubsystemBase {

  private SparkMax m_turretMotor = new SparkMax(RobotMap.kTurretMotorSparkMax, MotorType.kBrushless);
  private SparkRelativeEncoder m_turretEncoder;
  private SparkClosedLoopController m_turretController;
  private final SparkMaxConfig m_turretConfig;

  private static final double kStatic = 0.18;
  private static final PidValuesRecord pidValues = new PidValuesRecord(0.015, 0, 0);

  private final double kMaxTurretAngle = 110;
  private final double kMinTurrentAngle = -kMaxTurretAngle;
  private double m_requestedAngle = 0;

  private final NeoPidNetworkTableHelper m_networkTable = new NeoPidNetworkTableHelper("Turret", pidValues);
  private final DoublePublisher m_aprilTagTargetPub;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    m_turretEncoder = (SparkRelativeEncoder) m_turretMotor.getEncoder();
    m_turretController = m_turretMotor.getClosedLoopController();
    m_turretConfig = new SparkMaxConfig();
    m_turretConfig.idleMode(IdleMode.kCoast).inverted(true);

    // measured on turret that it took 25.6 motor rotations to turn 360 degrees
    m_turretConfig.encoder.positionConversionFactor(25.6 / 360.0);
    m_turretConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(pidValues.kP(), pidValues.kI(), pidValues.kD());
    m_turretConfig.closedLoop.feedForward.kS(kStatic);

    m_turretMotor.configure(m_turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // additional publisher
    var topic = m_networkTable.networkTable().getDoubleTopic("April Tag Angle");
    m_aprilTagTargetPub = topic.publish();
    
    goToTargetAngle(0);
  }

  int m_ticks = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_ticks++;
    if (m_ticks % 15 != 5)
      return;

    m_networkTable.dashboardUpdate(m_turretMotor, m_turretEncoder, m_turretConfig,
        (t) -> goToTargetAngle(t),
        (b) -> {});
    m_aprilTagTargetPub.set(AprilTagShooterHelpers.turretAngleToTarget(0));
  }

  public void stopTurret() {
    m_turretMotor.set(0);
  }

  public void goToTargetAngle(double targetAngle) {
    m_requestedAngle = keepAngleInOneEightySpace(targetAngle);
    if (m_requestedAngle < kMinTurrentAngle || m_requestedAngle > kMaxTurretAngle)
      return;
    m_turretController.setSetpoint(targetAngle, ControlType.kPosition);
  }

  private double keepAngleInOneEightySpace(double angle){
    // =IF(C20> 180, C20-360, IF(C20< -180, C20+360, C20))
    if (angle > 180)
      return angle - 360;
    if (angle < -180)
      return angle + 360;
    return angle;
  }

  public boolean isAtAngle(){
    return Math.abs(m_requestedAngle - m_turretEncoder.getPosition()) < 3;
  }
}
