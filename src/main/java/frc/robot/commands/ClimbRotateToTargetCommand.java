// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbRotateSubsystem;
import frc.robot.subsystems.ClimbRotateSubsystem.RotatePositionType;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbRotateToTargetCommand extends Command {

  private ClimbRotateSubsystem m_rotateSubsytem;
  private RotatePositionType m_rotatePosition;

  /** Creates a new ClimbRotateToTargetCommand. */
  public ClimbRotateToTargetCommand(ClimbRotateSubsystem rotateSubsystem, RotatePositionType rotatePosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rotateSubsystem);

    m_rotateSubsytem = rotateSubsystem;
    m_rotatePosition = rotatePosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotateSubsytem.setRotatePosition(m_rotatePosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_rotateSubsytem.holdAtCurrentPosition();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rotateSubsytem.isAtTargetPosition(m_rotatePosition);
  }
}
