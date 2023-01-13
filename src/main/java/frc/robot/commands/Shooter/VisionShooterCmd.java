// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.HoodAngleCmd;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;

public class VisionShooterCmd extends CommandBase {
  /** Creates a new VisionShooterCmd. */
  Shooter shooter;
  Hood m_hood;
  PhotonVision vision;

  public VisionShooterCmd(Shooter m_shooter, Hood hood, PhotonVision m_vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = m_shooter;
    this.vision = m_vision;
    this.m_hood = hood;
    addRequirements(shooter, m_hood, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("VisionShooterCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Burada hata var biliyorum fakat saat 04:10 olduğu için kafa pek çalışmıyor
    // burayı sequential veya parallel e almak lazım
    // çünkü hood vision ve shooter beraber çalışıp aynı anda işlem yapmalı

    if (vision.getDistance() < 10) {
      new HoodAngleCmd(m_hood, 0.5, 50);
      shooter.runShooter(0.3);
    } else if (vision.getDistance() >= 10 && vision.getDistance() < 20) {
      new HoodAngleCmd(m_hood, 0.5, 50);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("VisionShooterCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
