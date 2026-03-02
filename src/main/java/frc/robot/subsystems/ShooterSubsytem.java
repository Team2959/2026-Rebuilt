// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.robotarians.KrakenPidNetworkTableHelper;
import frc.robot.robotarians.PidValuesRecord;
import frc.robot.vision.AprilTagHelpers;

public class ShooterSubsytem extends SubsystemBase 
{
 public enum ShooterStateType
 {
  Off,
  Idle,
  PreptoShoot,
  Shooting
 }

  private TalonFX m_shooterWheel = new TalonFX(RobotMap.kShooterFollowerWheelkraken);
  private TalonFX m_shooterFollowerWheel = new TalonFX(RobotMap.kShooterFollowerWheelkraken);
  private Slot0Configs m_slot0Configs = new Slot0Configs();

  // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private VelocityVoltage m_velocityVoltage;

  private static final PidValuesRecord pidValues = new PidValuesRecord(1.0, 0.0, 0);

  private final KrakenPidNetworkTableHelper m_networkTable = new KrakenPidNetworkTableHelper("Shooter", pidValues);

  private final DoublePublisher m_aprilTagDistancePub;

  private ShooterStateType m_ShooterState = ShooterStateType.Off;
  private double m_requestedVelocity;

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

    m_shooterFollowerWheel.setNeutralMode(NeutralModeValue.Coast);
    m_shooterFollowerWheel.setControl(new Follower(RobotMap.kShooterFollowerWheelkraken, MotorAlignmentValue.Opposed));

    stopShooter();

    var topic = m_networkTable.networkTable().getDoubleTopic("April Tag Distance");
    m_aprilTagDistancePub = topic.publish();
  }

  int m_ticks = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_ticks++;
    if (m_ticks % 15 != 11)
      return;

    dashboardUpdate();
  }

  public void stopShooter() {
    m_shooterWheel.setControl(m_brake);
  }

  public void setVelocityfromDistance(double distance) {
    // ToDo: converts distance to velocity for accuracy
    var velocity = 5 + 25 * distance;
    // command each motor to control to the desired velocity
    setVelocity(velocity);
  }

  private void setVelocity(double target) {
    m_requestedVelocity = target;
    // current units are rotations per second
    m_shooterWheel.setControl(m_velocityVoltage.withVelocity(target));
    // m_shooterWheel.setControl(m_positionTorque.withPosition(bottom));
  }

  private void dashboardUpdate() {
    m_networkTable.dashboardUpdate(m_shooterWheel, m_slot0Configs, (t) -> setVelocity(t), (b) -> {});
    m_aprilTagDistancePub.set(AprilTagHelpers.distanceToTarget());
  }

  public Command shooterToIdleCommand() {
    return new InstantCommand(() -> shooterToIdle(), this);
  }

  public void shooterToIdle(){
    setVelocity(5.0);
    setShooterState(ShooterStateType.Idle);
  }

  public ShooterStateType getShooterState(){
    return m_ShooterState;
  }

  public void setShooterState(ShooterStateType newState){
    m_ShooterState = newState;
  }

  public boolean isAtVelocity(){
    return Math.abs(m_requestedVelocity - m_shooterWheel.getVelocity().getValueAsDouble()) < 0.5;
  }
}
