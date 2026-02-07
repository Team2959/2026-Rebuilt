// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax m_IntakeMotor = new SparkMax(RobotMap.kIntakeMotorSparkMax, MotorType.kBrushless);
 
  private static final double defaultSpeed = 0.5;
  private final DoubleSubscriber m_IntakeSpeedSub;
  private double m_IntakeSpeed = defaultSpeed;

  private static final double defaultReverseSpeed = -.5;
  private final DoubleSubscriber m_ReverseIntakeSpeedSub;
  private double m_reverseIntakeSpeed = defaultReverseSpeed;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("Intake");

    var speedTopic = datatable.getDoubleTopic("IntakeSpeed");
    speedTopic.publish().set(defaultSpeed);
    m_IntakeSpeedSub = speedTopic.subscribe(defaultSpeed);
  
    var reverseSpeedTopic = datatable.getDoubleTopic("ReverseIntakeSpeed");
    reverseSpeedTopic.publish().set(defaultSpeed);
    m_ReverseIntakeSpeedSub = reverseSpeedTopic.subscribe(defaultSpeed);
  }

  int m_ticks = 0;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
      m_ticks++;
    if (m_ticks % 15 != 3)
        return;
    
    dashboardUpdate();
  }

  private void startIntake(){
    m_IntakeMotor.set (m_IntakeSpeed);
  }

  private void stopIntake(){
    m_IntakeMotor.set(0);
  }

  private void reverseIntake(){
    m_IntakeMotor.set(m_reverseIntakeSpeed);
  }

  public Command reverseIntakeCommand(){
    return new StartEndCommand(() -> reverseIntake(), () -> stopIntake(), this);
  }

  public Command toggleIntakeCommand(){
    return new StartEndCommand(() -> startIntake(), () -> stopIntake(), this);
  }
      
  public void dashboardUpdate() {
    m_IntakeSpeed = m_IntakeSpeedSub.get();
    m_reverseIntakeSpeed = m_ReverseIntakeSpeedSub.get();
  }
}
