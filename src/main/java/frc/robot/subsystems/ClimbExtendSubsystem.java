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

public class ClimbExtendSubsystem extends SubsystemBase {
  public enum ExtendPositionType {
    Starting,
    LevelOnePrep,
    LevelOneEngage,
    LevelTwoPrep,
    LevelTwoEngage,
    LevelThreePrep,
    LevelThreeEngage
  }

  private TalonFX m_extendMotor = new TalonFX(RobotMap.kExtendLeftMotorkraken);
  private TalonFX m_extendRightMotor = new TalonFX(RobotMap.kExtendRightMotorkraken);
  private Slot0Configs m_extendConfig = new Slot0Configs();

  // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/PositionClosedLoop/src/main/java/frc/robot/Robot.java

  private PositionVoltage m_extendPositionVoltage;
  // private final PositionTorqueCurrentFOC m_positionTorque = new
  // PositionTorqueCurrentFOC(0).withSlot(1);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private static final PidValuesRecord pidValues = new PidValuesRecord(0.01, 0, 0);

  private final KrakenPidNetworkTableHelper m_networkTable = new KrakenPidNetworkTableHelper("Climb Extend", pidValues);

  /** Creates a new Shootersubsytem. */
  public ClimbExtendSubsystem() {
    m_extendPositionVoltage = new PositionVoltage(0);

    m_extendConfig.kS = 0.05; // Add 0.05 V output to overcome static friction
    m_extendConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    m_extendConfig.kP = pidValues.kP(); // An error of 1 rps results in 0.05 V output
    m_extendConfig.kI = pidValues.kI(); // no output for integrated error
    m_extendConfig.kD = pidValues.kD(); // no output for error derivative
    m_extendMotor.getConfigurator().apply(m_extendConfig);
    m_extendMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));
    m_extendMotor.setNeutralMode(NeutralModeValue.Brake);
    m_extendRightMotor.getConfigurator().apply(m_extendConfig);
    m_extendRightMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));
    m_extendRightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  int m_ticks = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_ticks++;
    if (m_ticks % 15 != 13)
      return;

    m_networkTable.dashboardUpdate(m_extendMotor, m_extendConfig, (t) -> setExtendPosition(t),
        (b) -> {
          m_extendRightMotor.getConfigurator().apply(m_extendConfig);
        });
  }

  public void stopShooter() {
    m_extendMotor.setControl(m_brake);
  }

  private double positionTypeToValue(ExtendPositionType position) {
    // in units of rotations
    switch (position) {
      case LevelOnePrep:
        return 10;
      case LevelOneEngage:
        return 8;
      case LevelTwoPrep:
        return 20;
      case LevelTwoEngage:
        return 18;
      case LevelThreePrep:
        return 30;
      case LevelThreeEngage:
        return 28;
      case Starting:
      default:
        // will keep target at 0
        return 0;
    }
  }

  public void setExtendPosition(ExtendPositionType position) {
    setExtendPosition(positionTypeToValue(position));
  }

  private void setExtendPosition(double position) {
    m_extendMotor.setControl(m_extendPositionVoltage.withPosition(position));
    m_extendRightMotor.setControl(m_extendPositionVoltage.withPosition(position));
    // m_extendMotor.setControl(m_positionTorque.withPosition(position));
  }

  public void holdAtCurrentPosition() {
    setExtendPosition(getCurrentPosition());
  }

  public boolean isAtTargetPosition(ExtendPositionType target) {
    return Math.abs(getCurrentPosition() - positionTypeToValue(target)) < 1;
  }

  private double getCurrentPosition() {
    return m_extendMotor.getPosition().getValueAsDouble();
  }
}
