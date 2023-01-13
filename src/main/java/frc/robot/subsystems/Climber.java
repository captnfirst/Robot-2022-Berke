// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climbers. */
  Spark fixedMotor;
  CANSparkMax movableLeftClimb;
  CANSparkMax movableRightClimb;
  MotorControllerGroup climbMotor;

  public Climber() {
    fixedMotor = new Spark(ClimberConstants.FIXED_CLIMBER_MOTOR_PWM);
    movableLeftClimb = new CANSparkMax(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushed);
    movableRightClimb = new CANSparkMax(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushed);
    climbMotor = new MotorControllerGroup(movableLeftClimb, movableRightClimb);
  }

  public void runClimb(double speed) {
    climbMotor.set(speed);
  }

  public void runRope(double m_speed) {
    fixedMotor.set(m_speed);
  }

  public void runLeft(double lSpeed){
    movableLeftClimb.set(lSpeed);
  }

  public void runRight(double rSpeed){
    movableRightClimb.set(rSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
