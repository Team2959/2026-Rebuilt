// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsytem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterVelocityfromDistanceCommand extends Command {

  private ShooterSubsytem m_ShooterSubsytem;

  /** Creates a new ShooterVelocityfromDistanceCommand. */
  public ShooterVelocityfromDistanceCommand(ShooterSubsytem shooterSubsytem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsytem);
    m_ShooterSubsytem = shooterSubsytem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get distance frame april tags
    // feed distance to shooter
    m_ShooterSubsytem.setVelocityfromDistance(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsytem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
