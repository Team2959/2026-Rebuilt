// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.robotarians.KrakenPidNetworkTableHelper;
import frc.robot.robotarians.PidValuesRecord;

public class ClimbRotateSubsystem extends SubsystemBase {
  public enum RotatePositionType{
    Starting,
    Tower,
    Retracted
  }

  private TalonFX m_rotateMotor = new TalonFX(RobotMap.kRotateMotorkraken);
  private Slot0Configs m_rotateConfig = new Slot0Configs();

  private PositionVoltage m_rotatePositionVoltage;
  // private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private static final PidValuesRecord pidValues = new PidValuesRecord(0.01, 0, 0);

  private final KrakenPidNetworkTableHelper m_networkTable = new KrakenPidNetworkTableHelper("Climb Rotate", pidValues);

  /** Creates a new Shootersubsytem. */
  public ClimbRotateSubsystem() {
     m_rotatePositionVoltage = new PositionVoltage(0);

    m_rotateConfig.kS = 0.05; // Add 0.05 V output to overcome static friction
    m_rotateConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    m_rotateConfig.kP = pidValues.kP(); // An error of 1 rps results in 0.05 V output
    m_rotateConfig.kI = pidValues.kI(); // no output for integrated error
    m_rotateConfig.kD = pidValues.kD(); // no output for error derivative
    m_rotateMotor.getConfigurator().apply(m_rotateConfig);
    m_rotateMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));
    m_rotateMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  int m_ticks = 0;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    m_ticks++;
    if (m_ticks % 15 != 2)
        return;
  
    m_networkTable.dashboardUpdate(m_rotateMotor, m_rotateConfig, (t) -> setRotatePosition(t), (b) -> {});
  }

  public void stopClimb(){
    m_rotateMotor.setControl(m_brake);
  }

  private double positionTypeToValue(RotatePositionType position){
    // units of rotations
    switch (position) {
      case Tower:
        return 10;
      case Retracted:
        return 2;
      case Starting:
      default:
        // will keep target at 0
        return 0;
      }
  }

  public void setRotatePosition(RotatePositionType position){
    setRotatePosition(positionTypeToValue(position));
  }

  private void setRotatePosition(double position){
    m_rotateMotor.setControl(m_rotatePositionVoltage.withPosition(position));
    // m_rotateMotor.setControl(m_positionTorque.withPosition(position));
  }

  public void holdAtCurrentPosition(){
    setRotatePosition(getCurrentPosition());
  }

  public boolean isAtTargetPosition(RotatePositionType target){
    return Math.abs(getCurrentPosition() - positionTypeToValue(target)) < 1;
  }

  private double getCurrentPosition()
  {
    return m_rotateMotor.getPosition().getValueAsDouble();
  }
}
