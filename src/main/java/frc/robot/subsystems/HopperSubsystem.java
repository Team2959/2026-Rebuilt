// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class HopperSubsystem extends SubsystemBase {

  private SparkMax m_HopperMotor = new SparkMax(RobotMap.kHopperMotorSparkMax, MotorType.kBrushless);

  private static final double defaultspeed = 1.0;
  private final DoubleSubscriber m_HopperSpeedSub;
  private double m_HopperSpeed = defaultspeed;

  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast)
        .inverted(true);

    m_HopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("Hopper");

    var speedTopic = datatable.getDoubleTopic("HopperSpeed");
    speedTopic.publish().set(defaultspeed);
    m_HopperSpeedSub = speedTopic.subscribe(defaultspeed);
  }

  int m_ticks = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_ticks++;
    if (m_ticks % 19 != 17)
      return;
    dashboardUpdate();
  }

  public void stopHopper() {
    m_HopperMotor.set(0);
  }

  public Command stopHopperCommand() {
    return new InstantCommand(() -> stopHopper());
  }

  private void startHopper() {
    m_HopperMotor.set(m_HopperSpeed);
  }

  public Command startHopperCommand() {
    return new InstantCommand(() -> startHopper());
  }

  private void reverseHopper() {
    m_HopperMotor.set(-m_HopperSpeed / 2.0);
  }

  public Command toggleHopperCommand() {
    return new StartEndCommand(() -> startHopper(), () -> stopHopper(), this);
  }

  public Command reverseHopperCommand() {
    return new StartEndCommand(() -> reverseHopper(), () -> stopHopper(), this);
  }

  public void dashboardUpdate() {
    m_HopperSpeed = m_HopperSpeedSub.get();
  }
}
