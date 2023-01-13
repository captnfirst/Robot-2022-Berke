// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  PhotonCamera camera;
  PhotonPipelineResult result;
  NetworkTableInstance NTmain;
  NetworkTable nt;
  public PhotonVision() {
    camera = new PhotonCamera("borusancam");
    NTmain = NetworkTableInstance.getDefault();
    nt = NTmain.getTable("photonvision").getSubTable("borusancam");
  }

  public boolean hasTargets() {
    return nt.getEntry("hasTargets").getBoolean(false);
  }

  public double getYaw() {
    return nt.getEntry("targetYaw").getDouble(Double.NaN);
  }

  public double getPitch() {
    return nt.getEntry("targetPitch").getDouble(Double.NaN);
  }

  public double getArea() {
    return nt.getEntry("targetArea").getDouble(Double.NaN);
  }

  public double getSkew() {
    return nt.getEntry("targetSkew").getDouble(Double.NaN);
  }

  public double getBestTarget() {
    return result.getBestTarget().getYaw();
  }

  public double getDistance(){
    double distance = (Vision.TARGET_HEIGHT_FROM_GROUND-Vision.CAMERA_HEIGHT) / Math.tan(Math.toRadians(getPitch() + Vision.CAMERA_PITCH));
    return distance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("X Koor.", getYaw());
    SmartDashboard.putNumber("Y Koor.", getPitch());
    SmartDashboard.putNumber("Alan", getArea());
    SmartDashboard.putNumber("Eğim", getSkew());
    SmartDashboard.putNumber("Uzaklık", getDistance());
    SmartDashboard.putBoolean("Has Targets", hasTargets());
  }
}
