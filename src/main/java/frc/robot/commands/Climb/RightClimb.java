// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RightClimb extends CommandBase {
  /** Creates a new RightClimb. */
  Climber climb;
  double rSpeed;

  public RightClimb(Climber m_climb, double m_speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = m_climb;
    this.rSpeed = m_speed;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("RightClimbCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.runRight(rSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("RightClimbCmd ended!");
    climb.runRight(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
