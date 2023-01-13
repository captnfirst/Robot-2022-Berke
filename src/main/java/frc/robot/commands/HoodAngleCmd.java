// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class HoodAngleCmd extends CommandBase {
  /** Creates a new HoodAngleCmd. */
  Hood hood;
  double speed;
  double angle;
  double lastval;
  boolean IsFinished = false;
  public HoodAngleCmd(Hood m_hood, double m_speed, double m_angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hood=m_hood;
    this.speed = m_speed;
    this.angle = m_angle;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastval= hood.getDistanceTraveled();
    if (angle>0) {
      while (hood.getDistanceTraveled()<lastval+angle) {
        hood.runHood(-(speed));
      }
    }
    else if (angle<0) {
      while (hood.getDistanceTraveled()>lastval+angle) {
        hood.runHood(speed);
      }
    }
    IsFinished=true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return IsFinished;
  }
}
