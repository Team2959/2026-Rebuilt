// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax m_IntakeMotor = new SparkMax(RobotMap.kIntakeMotorSparkMax, MotorType.kBrushless);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void startIntake(){
    m_IntakeMotor.set(1);
  }

  private void stopIntake(){
    m_IntakeMotor.set(0);
  }

  private void reverseIntake(){
    m_IntakeMotor.set(-.5);
  }

  public Command reverseIntakeCommand(){
    return new StartEndCommand(() -> reverseIntake(), () -> stopIntake(), this);
  }

  public Command toggleIntakeCommand(){
    return new StartEndCommand(() -> startIntake(), () -> stopIntake(), this);
  }

}
