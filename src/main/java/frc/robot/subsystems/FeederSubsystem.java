// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class FeederSubsystem extends SubsystemBase {

  private SparkMax m_FeederMotor = new SparkMax(RobotMap.kFeederMotorSparkMax, MotorType.kBrushless);

  private static final double defaultspeed = 0.5;
  private final DoubleSubscriber m_FeederSpeedSub;
  private double m_FeederSpeed = defaultspeed;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("Feeder");

    var speedTopic = datatable.getDoubleTopic("FeederSpeed");
    speedTopic.publish().set(defaultspeed);
    m_FeederSpeedSub = speedTopic.subscribe(defaultspeed);
  }

  int m_ticks = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_ticks++;
    if (m_ticks % 19 != 19)
      return;
    dashboardUpdate();
  }

  private void stopFeeder() {
    m_FeederMotor.set(0);
  }

  public Command stopfeederCommand() {
    return new InstantCommand(() -> stopFeeder(), this);
  }

  private void startFeeder() {
    m_FeederMotor.set(m_FeederSpeed);
  }

  public Command startfeederCommand() {
    return new InstantCommand(() -> startFeeder(), this);
  }

  public void dashboardUpdate() {
    m_FeederSpeed = m_FeederSpeedSub.get();
  }
}
