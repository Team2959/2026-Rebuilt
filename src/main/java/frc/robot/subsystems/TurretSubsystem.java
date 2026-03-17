// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.robotarians.NeoPidNetworkTableHelper;
import frc.robot.robotarians.PidValuesRecord;

public class TurretSubsystem extends SubsystemBase {

  private SparkMax m_turretMotor = new SparkMax(RobotMap.kTurretMotorSparkMax, MotorType.kBrushless);
  private SparkRelativeEncoder m_turretEncoder;
  private SparkClosedLoopController m_turretController;
  private final SparkMaxConfig m_turretConfig;

  private static final double kStatic = 0.18;// 0.22
  // initial testing had kP 0.015, but jerky at end
  private static final PidValuesRecord pidValues = new PidValuesRecord(0.1, 0, 0);
  private static final double kPositionConversionFactor = 360.0 / 25.6;

  private final double kMaxTurretAngle = 90;
  private final double kMinTurretAngle = -kMaxTurretAngle;
  private final double kTurretCrossoverBand = 45;
  private double m_requestedAngle = 0;
  private double m_rawRequest = 0;
  private boolean m_suspendAutoTurret = false;

  private final NeoPidNetworkTableHelper m_networkTable = new NeoPidNetworkTableHelper("Turret", pidValues);
  private final DoublePublisher m_requestTargetPub;
  private final DoublePublisher m_correctedTargetPub;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutAngle m_angle = Degrees.mutable(0);
  private final MutAngularVelocity m_velocity = DegreesPerSecond.mutable(0);
  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motor(s).
          m_turretMotor::setVoltage,
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // Record a frame for the shooter motor.
            log.motor("shooter-wheel")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        m_turretMotor.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(m_angle.mut_replace(m_turretEncoder.getPosition(), Degrees))
                .angularVelocity(
                    m_velocity.mut_replace(m_turretEncoder.getVelocity(), DegreesPerSecond));
          },
          // Tell SysId to make generated commands require this subsystem, suffix test
          // state in
          // WPILog with this subsystem's name ("shooter")
          this));

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    m_turretEncoder = (SparkRelativeEncoder) m_turretMotor.getEncoder();
    m_turretController = m_turretMotor.getClosedLoopController();
    m_turretConfig = new SparkMaxConfig();
    m_turretConfig.idleMode(IdleMode.kBrake).inverted(true);

    // measured on turret that it took 25.6 motor rotations to turn 360 degrees
    m_turretConfig.encoder.positionConversionFactor(kPositionConversionFactor)
        .velocityConversionFactor(kPositionConversionFactor / 60.0);
    m_turretConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(pidValues.kP(), pidValues.kI(), pidValues.kD());
    m_turretConfig.closedLoop.feedForward.kS(kStatic);// .kV(0.025).kA(0.025);
    // m_turretConfig.closedLoop.maxMotion.cruiseVelocity(120).maxAcceleration(75).allowedProfileError(3);

    m_turretMotor.configure(m_turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // additional publisher
    var topic = m_networkTable.networkTable().getDoubleTopic("Requested Angle");
    m_requestTargetPub = topic.publish();
    topic = m_networkTable.networkTable().getDoubleTopic("Corrected 180 Angle");
    m_correctedTargetPub = topic.publish();

    goToTargetAngle(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (RobotContainer.m_ticks % 15 != 5)
      return;

    m_networkTable.dashboardUpdate(m_turretMotor, m_turretEncoder, m_turretConfig,
        (t) -> goToTargetAngle(t, 0),
        (b) -> {
        });
    // m_requestTargetPub.set(m_rawRequest);
    m_correctedTargetPub.set(m_requestedAngle);
  }

  public void stopTurret() {
    m_turretMotor.set(0);
  }

  // private final double kDegreeLimiter = 20.0;

  public void goToTargetAngle(double targetAngle, double yawRate) {
    m_rawRequest = targetAngle;

    targetAngle = m_requestedAngle = keepAngleInOneEightySpace(targetAngle);

    var currentAngle = currentAngle();

    if (Math.abs(targetAngle - currentAngle) < 1.0)
      return;

      // if (Math.abs(targetAngle - currentAngle) > kDegreeLimiter) {
    // if (targetAngle > currentAngle)
    // targetAngle = currentAngle + kDegreeLimiter;
    // else
    // targetAngle = currentAngle - kDegreeLimiter;
    // }

    if (targetAngle > kMaxTurretAngle + kTurretCrossoverBand && (yawRate > 0)) {
      targetAngle = -kMaxTurretAngle;
    }

    if (targetAngle < -kMaxTurretAngle - kTurretCrossoverBand && (yawRate < 0)) {
      targetAngle = kMaxTurretAngle;
    }

    if (targetAngle < kMinTurretAngle || targetAngle > kMaxTurretAngle)
      return;
    m_turretController.setSetpoint(targetAngle, ControlType.kPosition);
    // m_turretController.setSetpoint(targetAngle,
    // ControlType.kMAXMotionPositionControl);
  }

  private double keepAngleInOneEightySpace(double angle) {
    // =IF(C20> 180, C20-360, IF(C20< -180, C20+360, C20))
    if (angle > 180)
      return angle - 360;
    if (angle < -180)
      return angle + 360;
    return angle;
  }

  public boolean isAtAngle() {
    return Math.abs(m_requestedAngle - currentAngle()) < 3;
  }

  public double currentAngle() {
    return m_turretEncoder.getPosition();
  }

  public boolean getSuspendAutoTurret() {
    return m_suspendAutoTurret;
  }

  public void setSuspendAutoTurret(boolean suspend) {
    m_suspendAutoTurret = suspend;
    if (suspend)
      m_turretController.setSetpoint(0, ControlType.kPosition);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
