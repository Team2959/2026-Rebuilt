// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsytem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsytem.ShooterStateType;

public class AutoFeedShooterCommand extends Command {
  private final FeederSubsystem m_feederSubsystem;
  private final ShooterSubsytem m_shooterSubsytem;
  private final TurretSubsystem m_turretSubsystem;

  /** Creates a new FeedShooterCommand. */
  public AutoFeedShooterCommand(FeederSubsystem feederSubsystem, ShooterSubsytem shooterSubsytem,
      TurretSubsystem turretSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feederSubsystem);
    m_feederSubsystem = feederSubsystem;
    m_shooterSubsytem = shooterSubsytem;
    m_turretSubsystem = turretSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterSubsytem.getShooterState() == ShooterStateType.PreptoShoot &&
        m_shooterSubsytem.isAtVelocity() && m_turretSubsystem.isAtAngle()) {
      m_feederSubsystem.startFeeder();
      m_shooterSubsytem.setShooterState(ShooterStateType.Shooting);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feederSubsystem.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_shooterSubsytem.isShooting();
  }
}
