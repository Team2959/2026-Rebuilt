// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.robotarians.KrakenPidNetworkTableHelper;
import frc.robot.robotarians.PidValuesRecord;

public class ShooterSubsytem extends SubsystemBase {

  // private TalonFX m_topShooterWheel = new TalonFX(RobotMap.kTopShooterWheelkraken);
  private TalonFX m_bottomShooterWheel = new TalonFX(RobotMap.kBottomShooterWheelkraken);
  private Slot0Configs m_slot0Configs = new Slot0Configs();

  // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  // private VelocityVoltage m_topVelocityVoltage;
  private VelocityVoltage m_bottomVelocityVoltage;

  private static final PidValuesRecord pidValues = new PidValuesRecord(0.05, 0.0, 0);

  private final KrakenPidNetworkTableHelper m_networkTable = new KrakenPidNetworkTableHelper("Shooter", pidValues);
  // private final DoubleSubscriber m_topTargetVelocitySub;
  // private final DoublePublisher m_topVelocityPub;
  // private final DoublePublisher m_topDutyCyclePublisher;

  /** Creates a new Shootersubsytem. */
  public ShooterSubsytem() {
    // m_topVelocityVoltage = new VelocityVoltage(0);
    m_bottomVelocityVoltage = new VelocityVoltage(0);

    m_slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
    m_slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    m_slot0Configs.kP = pidValues.kP(); // An error of 1 rps results in 0.05 V output
    m_slot0Configs.kI = pidValues.kI(); // no output for integrated error
    m_slot0Configs.kD = pidValues.kD(); // no output for error derivative
    // m_topShooterWheel.getConfigurator().apply(m_slot0Configs);
    // m_topShooterWheel.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));
    // m_topShooterWheel.setNeutralMode(NeutralModeValue.Coast);
    m_bottomShooterWheel.getConfigurator().apply(m_slot0Configs);
    m_bottomShooterWheel.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));
    m_bottomShooterWheel.setNeutralMode(NeutralModeValue.Coast);

    // var datatable = m_networkTable.networkTable();
    // var topTopic = datatable.getDoubleTopic("Top Target Velocity");
    // topTopic.publish().set(0);
    // m_topTargetVelocitySub = topTopic.subscribe(0);
    // m_topVelocityPub = datatable.getDoubleTopic("Top Velocity").publish();
    // m_topDutyCyclePublisher = datatable.getDoubleTopic("Top Duty Cycle").publish();
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
    // m_topShooterWheel.setControl(m_brake);
    m_bottomShooterWheel.setControl(m_brake);
  }

  public void setVelocityfromDistance(double distance) {
    // ToDo: converts distance to velocity for accuracy
    // command each motor to control to the desired velocity
    setVelocity(0, 0);
  }

  private void setVelocity(double top, double bottom) {
    // current units are rotations per second
    // m_topShooterWheel.setControl(m_topVelocityVoltage.withVelocity(top));
    m_bottomShooterWheel.setControl(m_bottomVelocityVoltage.withVelocity(bottom));

    // m_topShooterWheel.setControl(m_positionTorque.withPosition(top));
  }

  private void dashboardUpdate() {
    // m_topVelocityPub.set(m_topShooterWheel.getVelocity().getValueAsDouble());
    // m_topDutyCyclePublisher.set(m_topShooterWheel.getDutyCycle().getValueAsDouble());
    m_networkTable.dashboardUpdate(m_bottomShooterWheel, m_slot0Configs, (t) -> setBothVelocities(t), (b) -> updateTopConfigs());
  }

  private void updateTopConfigs(){
      // m_topShooterWheel.getConfigurator().apply(m_slot0Configs);
  }

  private void setBothVelocities(Double bottomTarget){
      // setVelocity(m_topTargetVelocitySub.get(), bottomTarget);
      setVelocity(0, bottomTarget);
  }
}
