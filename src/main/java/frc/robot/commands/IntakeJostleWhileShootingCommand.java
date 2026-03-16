// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ExtendIntakePositionType;

public class IntakeJostleWhileShootingCommand extends Command {
  private IntakeSubsystem m_intakeSubsystem;
  private boolean m_up = true;
  private int m_delayTicks;

  /** Creates a new IntakeJostleWhileShootingCommand. */
  public IntakeJostleWhileShootingCommand(IntakeSubsystem intakeSubsystem) {
    addRequirements(m_intakeSubsystem);
    m_intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.setExtendPosition(ExtendIntakePositionType.JostleUp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_delayTicks > 0) {
      m_delayTicks++;
      if (m_delayTicks > 10) {
        m_delayTicks = 0;
        m_intakeSubsystem
            .setExtendPosition(m_up ? ExtendIntakePositionType.JostleDown : ExtendIntakePositionType.JostleUp);
        m_up = !m_up;
      }
      return;
    }

    if ((m_up && m_intakeSubsystem.isAtPosition(ExtendIntakePositionType.JostleUp)) ||
        (!m_up && m_intakeSubsystem.isAtPosition(ExtendIntakePositionType.JostleDown))) {
      m_delayTicks = 1;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setExtendPosition(ExtendIntakePositionType.Extended);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
