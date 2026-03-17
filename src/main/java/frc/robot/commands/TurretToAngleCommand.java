// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretToAngleCommand extends Command {
  TurretSubsystem m_turretSubsystem;
  double m_target;

  /** Creates a new TurrettoAngleCommand. */
  public TurretToAngleCommand(TurretSubsystem turretSubsystem, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
    m_turretSubsystem = turretSubsystem;
    m_target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turretSubsystem.goToTargetAngle(m_target, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      m_turretSubsystem.goToTargetAngle(m_turretSubsystem.currentAngle(), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turretSubsystem.isAtAngle();
  }
}
