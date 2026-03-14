// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsytem;
import frc.robot.subsystems.ShooterSubsytem.ShooterStateType;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterVelocityfromDistanceCommand extends Command {

  private ShooterSubsytem m_ShooterSubsytem;
  private Supplier<Double> m_targetDistance;

  /** Creates a new ShooterVelocityfromDistanceCommand. */
  public ShooterVelocityfromDistanceCommand(ShooterSubsytem shooterSubsytem, Supplier<Double> targetDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsytem);
    m_ShooterSubsytem = shooterSubsytem;
    m_targetDistance = targetDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsytem.setShooterState(ShooterStateType.PreptoShoot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ShooterSubsytem.getFixedShooterSpeed()) {
      m_ShooterSubsytem.setVelocity(m_ShooterSubsytem.getFixedSpeed());
    } else {
      // get distance frame april tags
      // feed distance to shooter
      // var distance = AprilTagShooterHelpers.distanceToTarget();
      var distance = m_targetDistance.get();
      if (Double.isNaN(distance))
        return;
      m_ShooterSubsytem.setVelocityfromDistance(distance);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsytem.shooterToIdle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_ShooterSubsytem.isShooting();
  }
}
