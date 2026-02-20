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

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsytem extends SubsystemBase {

  private TalonFX m_topShooterWheel = new TalonFX(RobotMap.kTopShooterWheelkraken);
  private TalonFX m_bottomShooterWheel = new TalonFX(RobotMap.kBottomShooterWheelkraken);
  private Slot0Configs m_slot0Configs = new Slot0Configs();

  // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private VelocityVoltage m_topVelocityVoltage;
  private VelocityVoltage m_bottomVelocityVoltage;

  private static final double kP = 0.0005;
  private static final double kI = 0.000001;
  private static final double kD = 0.0;
  private static final double KFf = 0.0;

  private final DoubleSubscriber m_kPSub;
  private final DoubleSubscriber m_kISub;
  private final DoubleSubscriber m_kDSub;
  private final DoubleSubscriber m_kFfSub;
  private final DoubleSubscriber m_topTargetVelocitySub;
  private final DoubleSubscriber m_bottomTargetVelocitySub;
  private final DoublePublisher m_topVelocityPub;
  private final DoublePublisher m_bottomVelocityPub;

  private final BooleanSubscriber m_goToTargetSub;
  private final BooleanPublisher m_goToTargetPub;
  private final BooleanSubscriber m_updatePidSub;
  private final BooleanPublisher m_updatePidPub;

  private final DoublePublisher m_topAppliedOutputPublisher;
  private final DoublePublisher m_bottomAppliedOutputPublisher;

  /** Creates a new Shootersubsytem. */
  public ShooterSubsytem() {
    m_topVelocityVoltage = new VelocityVoltage(0);
    m_bottomVelocityVoltage = new VelocityVoltage(0);

    m_slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
    m_slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    m_slot0Configs.kP = 0.05; // An error of 1 rps results in 0.05 V output
    m_slot0Configs.kI = 0; // no output for integrated error
    m_slot0Configs.kD = 0; // no output for error derivative
    m_topShooterWheel.getConfigurator().apply(m_slot0Configs);
    m_topShooterWheel.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));
    m_topShooterWheel.setNeutralMode(NeutralModeValue.Coast);
    m_bottomShooterWheel.getConfigurator().apply(m_slot0Configs);
    m_bottomShooterWheel.getConfigurator().apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.100));
    m_bottomShooterWheel.setNeutralMode(NeutralModeValue.Coast);

    // get the subtable called "serveMod1"
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("Shooter");

    // PID topic
    var kpTopic = datatable.getDoubleTopic("kP");
    kpTopic.publish().set(kP);
    m_kPSub = kpTopic.subscribe(kP);
    var kiTopic = datatable.getDoubleTopic("kI");
    kiTopic.publish().set(kI);
    m_kISub = kiTopic.subscribe(kI);
    var kDTopic = datatable.getDoubleTopic("kD");
    kDTopic.publish().set(kD);
    m_kDSub = kDTopic.subscribe(kD);
    var ffTopic = datatable.getDoubleTopic("FF");
    ffTopic.publish().set(KFf);
    m_kFfSub = ffTopic.subscribe(KFf);

    var topTopic = datatable.getDoubleTopic("Top Target Velocity");
    topTopic.publish().set(0);
    m_topTargetVelocitySub = topTopic.subscribe(0);

    var bottomTopic = datatable.getDoubleTopic("Bottom Target Velocity");
    bottomTopic.publish().set(0);
    m_bottomTargetVelocitySub = bottomTopic.subscribe(0);

    m_topVelocityPub = datatable.getDoubleTopic("Top Velocity").publish();
    m_bottomVelocityPub = datatable.getDoubleTopic("Bottom Velocity").publish();

    var goToTarget = datatable.getBooleanTopic("go to Target");
    m_goToTargetPub = goToTarget.publish();
    m_goToTargetPub.set(false);
    m_goToTargetSub = goToTarget.subscribe(false);

    var updatePID = datatable.getBooleanTopic("update PID");
    m_updatePidPub = updatePID.publish();
    m_updatePidPub.set(false);
    m_updatePidSub = updatePID.subscribe(false);

    m_topAppliedOutputPublisher = datatable.getDoubleTopic("Top Applied Output").publish();
    m_bottomAppliedOutputPublisher = datatable.getDoubleTopic("Bottom Applied Output").publish();
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
    m_topShooterWheel.setControl(m_brake);
    m_bottomShooterWheel.setControl(m_brake);
  }

  public void setVelocityfromDistance(double distance) {
    // ToDo: converts distance to velocity for accuracy
    // command each motor to control to the desired velocity
    setVelocity(0, 0);
  }

  private void setVelocity(double top, double bottom) {
    // current units are rotations per second
    m_topShooterWheel.setControl(m_topVelocityVoltage.withVelocity(top));
    m_bottomShooterWheel.setControl(m_bottomVelocityVoltage.withVelocity(bottom));

    // m_topShooterWheel.setControl(m_positionTorque.withPosition(top));
}

  private void dashboardUpdate() {
    // m_topAppliedOutputPublisher.set(m_topShooterWheel;

    m_topVelocityPub.set(m_topShooterWheel.getVelocity().getValueAsDouble());
    m_bottomVelocityPub.set(m_bottomShooterWheel.getVelocity().getValueAsDouble());

    if (m_updatePidSub.get()) {
      double kp = m_kPSub.get();
      double ki = m_kISub.get();
      double kd = m_kDSub.get();

      m_slot0Configs.kP = kp;
      m_slot0Configs.kI = ki;
      m_slot0Configs.kD = kd;

      m_topShooterWheel.getConfigurator().apply(m_slot0Configs);
      m_bottomShooterWheel.getConfigurator().apply(m_slot0Configs);

      m_updatePidPub.set(false);
    }

    if (m_goToTargetSub.get()) {
      setVelocity(m_topTargetVelocitySub.get(), m_bottomTargetVelocitySub.get());

      m_goToTargetPub.set(false);
    }
  }
}
