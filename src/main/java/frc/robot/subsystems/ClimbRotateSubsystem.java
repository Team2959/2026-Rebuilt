// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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

public class ClimbRotateSubsystem extends SubsystemBase {
  public enum RotatePositionType{
    Starting,
    Tower,
    Retracted
  }

  private TalonFX m_rotateMotor = new TalonFX(RobotMap.kRotateMotorkraken);
  private Slot0Configs m_rotateConfig = new Slot0Configs();

  private PositionVoltage m_rotatePositionVoltage;

  private static final double rotateKp = 0.01;
  private static final double rotateKi = 0.0;
  private static final double rotateKd = 0.0;

  private final DoubleSubscriber m_kPSub;
  private final DoubleSubscriber m_kISub;
  private final DoubleSubscriber m_kDSub;
  private final DoubleSubscriber m_rotateTargetSub;
  private final DoublePublisher m_rotatePositionPub;

  private final BooleanSubscriber m_goToTargetSub;
  private final BooleanPublisher m_goToTargetPub;
  private final BooleanSubscriber m_updatePidSub;
  private final BooleanPublisher m_updatePidPub;

  private final DoublePublisher m_RotateAppliedOutputPublisher;

  /** Creates a new Shootersubsytem. */
  public ClimbRotateSubsystem() {
     m_rotatePositionVoltage = new PositionVoltage(0);

    m_rotateConfig.kS = 0.05; // Add 0.05 V output to overcome static friction
    m_rotateConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    m_rotateConfig.kP = rotateKp; // An error of 1 rps results in 0.05 V output
    m_rotateConfig.kI = rotateKi; // no output for integrated error
    m_rotateConfig.kD = rotateKd; // no output for error derivative
    m_rotateMotor.getConfigurator().apply(m_rotateConfig);
    m_rotateMotor.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));
    m_rotateMotor.setNeutralMode(NeutralModeValue.Coast);
    
    // get the subtable called "serveMod1"
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("Climb Rotate");

    // PID topic 
    var kpTopic = datatable.getDoubleTopic("kP");
    kpTopic.publish().set(rotateKp);
    m_kPSub = kpTopic.subscribe(rotateKp);
    var kiTopic = datatable.getDoubleTopic("kI");
    kiTopic.publish().set(rotateKi);
    m_kISub = kiTopic.subscribe(rotateKi);
    var kDTopic = datatable.getDoubleTopic("kD");
    kDTopic.publish().set(rotateKd);
    m_kDSub = kDTopic.subscribe(rotateKd);

    var rotateTargetTopic = datatable.getDoubleTopic("Rotate Target");
    rotateTargetTopic.publish().set(0);
    m_rotateTargetSub = rotateTargetTopic.subscribe(0);
    
    m_rotatePositionPub = datatable.getDoubleTopic("Rotate Position").publish();

    var goToTarget = datatable.getBooleanTopic("go to Target");
    m_goToTargetPub = goToTarget.publish();
    m_goToTargetPub.set(false);
    m_goToTargetSub = goToTarget.subscribe(false);

    var updatePID = datatable.getBooleanTopic("update PID");
    m_updatePidPub = updatePID.publish();
    m_updatePidPub.set(false);
    m_updatePidSub = updatePID.subscribe(false);

    m_RotateAppliedOutputPublisher = datatable.getDoubleTopic("Rotate Applied Output").publish();
  }

  int m_ticks = 0;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    m_ticks++;
    if (m_ticks % 15 != 2)
        return;
  
    dashboardUpdate();
  }

  public void stopClimb(){
    m_rotateMotor.set(0);
  }

  private double positionTypeToValue(RotatePositionType position){
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

  private void dashboardUpdate(){
    // m_topAppliedOutputPublisher.set(m_topShooterWheel;

    m_rotatePositionPub.set(getCurrentPosition());

    if (m_updatePidSub.get()) {
      double kp = m_kPSub.get();
      double ki = m_kISub.get();
      double kd = m_kDSub.get();

      m_rotateConfig.kP = kp;
      m_rotateConfig.kI = ki;
      m_rotateConfig.kD = kd;

      m_rotateMotor.getConfigurator().apply(m_rotateConfig);

      m_updatePidPub.set(false);
    }

    if (m_goToTargetSub.get()) {
      setRotatePosition(m_rotateTargetSub.get());

      m_goToTargetPub.set(false);
    }
  }
}
