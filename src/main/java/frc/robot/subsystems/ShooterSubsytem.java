// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

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

  private double m_targetTopSpeed = 0;
  private double m_targetBottomSpeed = 0;

  private final DoublePublisher m_topAppliedOutputPublisher;
  private final DoublePublisher m_bottomAppliedOutputPublisher;


  /** Creates a new Shootersubsytem. */
  public ShooterSubsytem() {

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
