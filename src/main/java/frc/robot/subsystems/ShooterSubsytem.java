// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.robotarians.KrakenPidNetworkTableHelper;
import frc.robot.robotarians.PidValuesRecord;

public class ShooterSubsytem extends SubsystemBase {
  public enum ShooterStateType {
    Off,
    Idle,
    PreptoShoot,
    Shooting
  }

  private TalonFX m_shooterWheel = new TalonFX(RobotMap.kShooterPrimaryWheelkraken);
  private TalonFX m_shooterFollowerWheel = new TalonFX(RobotMap.kShooterFollowerWheelkraken);
  private Slot0Configs m_slot0Configs = new Slot0Configs();
  private VelocityVoltage m_velocityVoltage;
  private final PidValuesRecord pidValues = new PidValuesRecord(1.0, 0.0, 0);

  // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private final KrakenPidNetworkTableHelper m_networkTable = new KrakenPidNetworkTableHelper("Shooter", pidValues);
  private final DoubleSubscriber m_fixedSpeedSub;

  private ShooterStateType m_ShooterState = ShooterStateType.Off;
  private double m_requestedVelocity;

  private final double kIdleSpeed = 10.0;
  public final double k2MeterSpeed = 35;//42.5;
  private boolean m_fixedShooterSpeed;
  private double m_fixedSpeed = k2MeterSpeed;

  /** Creates a new Shootersubsytem. */
  public ShooterSubsytem() {
    m_velocityVoltage = new VelocityVoltage(0);

    m_slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
    m_slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    m_slot0Configs.kP = pidValues.kP(); // An error of 1 rps results in 0.05 V output
    m_slot0Configs.kI = pidValues.kI(); // no output for integrated error
    m_slot0Configs.kD = pidValues.kD(); // no output for error derivative

    m_shooterWheel.getConfigurator().apply(m_slot0Configs);
    m_shooterWheel.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));
    m_shooterWheel.setNeutralMode(NeutralModeValue.Coast);
    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    motorConfigs.withInverted(InvertedValue.CounterClockwise_Positive);
    m_shooterWheel.getConfigurator().apply(motorConfigs);

    m_shooterFollowerWheel.setNeutralMode(NeutralModeValue.Coast);
    m_shooterFollowerWheel.setControl(new Follower(RobotMap.kShooterPrimaryWheelkraken, MotorAlignmentValue.Opposed));

    stopShooter();

    var topic = m_networkTable.networkTable().getDoubleTopic("Fixed Speed");
    topic.publish().set(m_fixedSpeed);
    m_fixedSpeedSub = topic.subscribe(m_fixedSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if (RobotContainer.m_ticks % 15 != 11)
    //   return;

    // m_networkTable.dashboardUpdate(m_shooterWheel, m_slot0Configs, (t) ->
    // setVelocity(t), (b) -> {
    // });

    // m_fixedSpeed = m_fixedSpeedSub.get();
  }

  public void stopShooter() {
    m_shooterWheel.setControl(m_brake);
  }

  public void setVelocityfromDistance(double distance) {
    var velocity = velocityFromDistance(distance);
    // command each motor to control to the desired velocity
    setVelocity(velocity);
  }

  public void setVelocity(double target) {
    m_requestedVelocity = target;
    // current units are rotations per second
    m_shooterWheel.setControl(m_velocityVoltage.withVelocity(target));
  }

  public Command shooterToIdleCommand() {
    return new InstantCommand(() -> shooterToIdle());
  }

  public void shooterToIdle() {
    setVelocity(getFixedShooterSpeed() ? getFixedSpeed() : kIdleSpeed);
    setShooterState(ShooterStateType.Idle);
  }

  public boolean isShooting() {
    var state = getShooterState();
    return state == ShooterStateType.PreptoShoot || state == ShooterStateType.Shooting;
  }

  public ShooterStateType getShooterState() {
    return m_ShooterState;
  }

  public void setShooterState(ShooterStateType newState) {
    m_ShooterState = newState;
  }

  public boolean isAtVelocity() {
    var velocity = m_shooterWheel.getVelocity().getValueAsDouble();
    return velocity > 35 && velocity - m_requestedVelocity > -1.0;
  }

  private double velocityFromDistance(double distance) {
    if (distance <= 2)
      return k2MeterSpeed;
    var lowerSpeed = k2MeterSpeed;
    var upperSpeed = 46.0;
    var lowerDistance = 2.0;
    if (distance >= 5) {
      return Math.max(105, 66 + (distance - 5) * 20);
    } else if (distance >= 4.5) {
      upperSpeed = 66;
      lowerSpeed = 63;
      lowerDistance = 3.5;
    } else if (distance >= 4) {
      upperSpeed = 63;
      lowerSpeed = 59.5;
      lowerDistance = 3.5;
    } else if (distance >= 3.5) {
      upperSpeed = 59.5;
      lowerSpeed = 56;
      lowerDistance = 3.5;
    } else if (distance >= 3) {
      lowerSpeed = 52;
      upperSpeed = 56;
      lowerDistance = 3;
    } else if (distance >= 2.5) {
      lowerSpeed = 46;
      upperSpeed = 52;
      lowerDistance = 2.5;
    }

    var deltaSpeed = upperSpeed - lowerSpeed;
    return lowerSpeed + (distance - lowerDistance) * deltaSpeed / 0.5;
  }

  public boolean getFixedShooterSpeed() {
    return m_fixedShooterSpeed;
  }

  public void setFixedShooterSpeed(boolean suspend) {
    m_fixedShooterSpeed = suspend;
  }

  public double getFixedSpeed() {
    return m_fixedSpeed;
  }

  public void setFixedSpeed(double speed) {
    m_fixedSpeed = speed;
  }
}
