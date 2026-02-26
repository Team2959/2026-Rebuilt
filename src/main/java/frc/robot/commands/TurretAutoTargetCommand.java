// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.vision.AprilTagHelpers;

public class TurretAutoTargetCommand extends Command {
  private final TurretSubsystem m_turretSubsystem;
  private Supplier<Double> m_robotAngleSupplier;

  /** Creates a new TurretAutoTarget. */
  public TurretAutoTargetCommand(TurretSubsystem turretSubsystem, Supplier<Double> robotAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
    m_turretSubsystem = turretSubsystem;
    m_robotAngleSupplier = robotAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ToDo: be able to target more than just the hub
    var target = AprilTagHelpers.turretAngleToHub(m_robotAngleSupplier.get());
    if (Double.isNaN(target))
      return;
    m_turretSubsystem.goToTargetAngle(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
