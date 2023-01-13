// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  PWMTalonSRX hoodMotor;
  Encoder encoder;
  double whd = 3 / 2.54;
  double cpr = 7;
  double hoodWheel = 35 / 2.54;

  public Hood() {
    hoodMotor = new PWMTalonSRX(HoodConstants.HOOD_MOTOR_PWM);
    encoder = new Encoder(HoodConstants.channelA, HoodConstants.channelB, HoodConstants.reverseDirection);
  }

  public void runHood(double speed) {
    hoodMotor.set(speed);
  }

  public void resetEncoder() {
    encoder.reset();
  }

  public double getDistanceTraveled() {
    // distance per pulse is pi* (wheel diameter / counts per revolution)
    encoder.setDistancePerPulse(Math.PI * whd / cpr);
    return encoder.getDistance();

    /*
     * buraya bizim açıyı getirmemiz lazım bir şekilde ve belirli bir açı vermemiz
     * lazım onu da robotu aşağıya
     * indirip bulmamız lazım mesela 2021 de team254 45-70 arasında açılarda hood
     * unu aşağı yukarı yaptırıyormuş
     */

    // return encoder.getDistance() * (whd / hoodWheel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Encoder", getDistanceTraveled());
  }
}
