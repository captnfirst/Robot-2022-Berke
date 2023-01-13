// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class VisionX extends CommandBase {
  /** Creates a new VisionX. */
  DriveTrain drive;
  PhotonVision vision;
  double xSpeed;
  double zRotation;
  double ANGULAR_P = 0.1;
  double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  public VisionX(DriveTrain m_drive, PhotonVision m_vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = m_drive;
    this.vision = m_vision;
    addRequirements(drive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("VisionX started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.hasTargets()) {
      zRotation = -turnController.calculate(vision.getBestTarget(), 0);
    } else {
      zRotation = 0;
    }
    drive.arcadeDrive(xSpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("VisionX ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
