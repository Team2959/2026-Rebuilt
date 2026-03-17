// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAutoTargetCommand extends Command {
  private final TurretSubsystem m_turretSubsystem;
  private Supplier<Double> m_targetAngle;
  private Supplier<Double> m_yawRate;

  /** Creates a new TurretAutoTarget. */
  public TurretAutoTargetCommand(TurretSubsystem turretSubsystem,
   Supplier<Double> targetAngle,
   Supplier<Double> yawRate) {
    addRequirements(turretSubsystem);
    m_turretSubsystem = turretSubsystem;
    m_targetAngle = targetAngle;
    m_yawRate = yawRate;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_turretSubsystem.getSuspendAutoTurret()) {
      m_turretSubsystem.goToTargetAngle(0, 0);
      return;
    }

    // MegaTag2 targeting
    var mt2Target = m_targetAngle.get();
    if (!Double.isNaN(mt2Target))
      m_turretSubsystem.goToTargetAngle(mt2Target, m_yawRate.get());

    // Robotarians targeting used at Lakeview
    // var target =
    // AprilTagShooterHelpers.turretAngleToTarget(m_driveSubsystem.getModuloAngle(),
    // m_isShooting.get());
    // if (Double.isNaN(target))
    // return;
    // m_turretSubsystem.setMt2Target(target);
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
