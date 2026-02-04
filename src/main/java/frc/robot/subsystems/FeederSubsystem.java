// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class FeederSubsystem extends SubsystemBase {

  private SparkMax m_FeederMotor = new SparkMax(RobotMap.kFeederMotorSparkMax, MotorType.kBrushless);

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopFeeder(){
    m_FeederMotor.set(0);
  }

  public void startFeeder(){
    m_FeederMotor.set(0.5);
  }
}
