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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.robotarians.NeoPidNetworkTableHelper;

public class TurretSubsystem extends SubsystemBase {

  private SparkMax m_turretMotor = new SparkMax(RobotMap.kTurretMotorSparkMax, MotorType.kBrushless);
  private SparkRelativeEncoder m_turretEncoder;
  private SparkClosedLoopController m_turretController;
  private final SparkMaxConfig m_turretConfig;

  private static final double kP = 0.0005;
  private static final double kI = 0.000001;
  private static final double kD = 0.0;

  private final NeoPidNetworkTableHelper m_networkTable = new NeoPidNetworkTableHelper("Turret", kP, kI, kD);

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
  }

  int m_ticks = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_ticks++;
    if (m_ticks % 15 != 5)
      return;

    m_networkTable.dashboardUpdate(m_turretMotor, m_turretEncoder, m_turretConfig, (t) -> goToTargetAngle(t), (b) -> {});
  }

  public void stopTurret() {
    m_turretMotor.set(0);
  }

  public void goToTargetAngle(double targetAngle) {
    // currently using rotations
    // ToDo: limit input to min/max allowed angles, if out of range, don't change
    // the target
    // ToDo: change to using degrees as the input, have a conversion from degrees to
    // position
    m_turretController.setSetpoint(targetAngle, ControlType.kPosition);
  }
}
