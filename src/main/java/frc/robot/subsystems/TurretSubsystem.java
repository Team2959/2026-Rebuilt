// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class TurretSubsystem extends SubsystemBase {

  private SparkMax m_turretMotor = new SparkMax(RobotMap.kTurretMotorSparkMax, MotorType.kBrushless);
  private SparkRelativeEncoder m_turretEncoder;
  private SparkClosedLoopController m_turretController;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {    
    m_turretEncoder = (SparkRelativeEncoder) m_turretMotor.getEncoder();          
    m_turretController = m_turretMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopTurret(){
    m_turretMotor.set(0);
  }

  public void goToTargetAngle(double targetAngle){
    m_turretController.setSetpoint(targetAngle, ControlType.kPosition);
  }
}
