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

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

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

  private TalonFX m_extendMotor = new TalonFX(RobotMap.kBottomShooterWheelkraken);
  private Slot0Configs m_extendConfig = new Slot0Configs();

  // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/PositionClosedLoop/src/main/java/frc/robot/Robot.java
  
  private PositionVoltage m_extendPositionVoltage;
  // private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

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

  private final DoublePublisher m_extendDutyCyclePub;

  /** Creates a new Shootersubsytem. */
  public ClimbExtendSubsystem() {
    m_extendPositionVoltage = new PositionVoltage(0);

    m_extendConfig.kS = 0.05; // Add 0.05 V output to overcome static friction
    m_extendConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    m_extendConfig.kP = extendKp; // An error of 1 rps results in 0.05 V output
    m_extendConfig.kI = extendKi; // no output for integrated error
    m_extendConfig.kD = extendKd; // no output for error derivative
    m_extendMotor.getConfigurator().apply(m_extendConfig);
    m_extendMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));
    m_extendMotor.setNeutralMode(NeutralModeValue.Coast);

    // get the subtable called "serveMod1"
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("Climb");

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

    var extendTargetTopic = datatable.getDoubleTopic("Extend Target");
    extendTargetTopic.publish().set(0);
    m_extendTargetSub = extendTargetTopic.subscribe(0);

    m_extendPositionPub = datatable.getDoubleTopic("Extend Position").publish();

    var goToTarget = datatable.getBooleanTopic("go to Target");
    m_goToTargetPub = goToTarget.publish();
    m_goToTargetPub.set(false);
    m_goToTargetSub = goToTarget.subscribe(false);

    var updatePID = datatable.getBooleanTopic("update PID");
    m_updatePidPub = updatePID.publish();
    m_updatePidPub.set(false);
    m_updatePidSub = updatePID.subscribe(false);

    m_extendDutyCyclePub = datatable.getDoubleTopic("Extend Duty Cycle").publish();
  }

  int m_ticks = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_ticks++;
    if (m_ticks % 15 != 13)
      return;

    dashboardUpdate();
  }

  public void stopShooter() {
    m_extendMotor.setControl(m_brake);
  }

  private double positionTypeToValue(ExtendPositionType position) {
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

  private void dashboardUpdate() {
    m_extendPositionPub.set(getCurrentPosition());
    m_extendDutyCyclePub.set(m_extendMotor.getDutyCycle().getValueAsDouble());

    if (m_updatePidSub.get()) {
      m_extendConfig.kP = m_kPSub.get();
      m_extendConfig.kI = m_kISub.get();
      m_extendConfig.kD = m_kDSub.get();
      m_extendMotor.getConfigurator().apply(m_extendConfig);

      m_updatePidPub.set(false);
    }

    if (m_goToTargetSub.get()) {
      setExtendPosition(m_extendTargetSub.get());

      m_goToTargetPub.set(false);
    }
  }
}
