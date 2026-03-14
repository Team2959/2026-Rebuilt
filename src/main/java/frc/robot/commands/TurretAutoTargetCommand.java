// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.vision.AprilTagShooterHelpers;

public class TurretAutoTargetCommand extends Command {
  private final TurretSubsystem m_turretSubsystem;
  private DriveSubsystem m_driveSubsystem;
  private Supplier<Boolean> m_isShooting;

  /** Creates a new TurretAutoTarget. */
  public TurretAutoTargetCommand(TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem, Supplier<Boolean> isShooting) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
    m_turretSubsystem = turretSubsystem;
    m_driveSubsystem = driveSubsystem;
    m_isShooting = isShooting;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_turretSubsystem.getSuspendAutoTurret()) return;
    AprilTagShooterHelpers.updateLimelightPose(m_turretSubsystem.currentAngle());
    AprilTagShooterHelpers.updateRobotOrientation(m_driveSubsystem.getAngle().getDegrees(), m_driveSubsystem.getYawRate());
    var mt2Target = AprilTagShooterHelpers.mt2TargetAngle();
    if (!Double.isNaN(mt2Target))
      m_turretSubsystem.goToTargetAngle(mt2Target);
    var target = AprilTagShooterHelpers.turretAngleToTarget(m_driveSubsystem.getModuloAngle(), m_isShooting.get());
    if (Double.isNaN(target))
      return;
    m_turretSubsystem.setMt2Target(target);
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
