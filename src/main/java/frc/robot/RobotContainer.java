// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.HoodAngleCmd;
import frc.robot.commands.IndexerCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.TriggerCmd;
import frc.robot.commands.Shooter.VisionShooterCmd;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  public final static DriveTrain m_driveTrain = new DriveTrain();
  public final static Intake m_intake = new Intake();
  public final static Indexer m_indexer = new Indexer();
  public final static Trigger m_trigger = new Trigger();
  public final static Shooter m_shooter = new Shooter();
  public final static Hood m_hood = new Hood();
  public final static PhotonVision m_vision = new PhotonVision();

  // Commands
  public static HoodAngleCmd m_hoodangle = new HoodAngleCmd(m_hood, 0.5, 50);

  // Joysticks
  public static Joystick JOYSTICK = new Joystick(JoystickConstants.JOYSTICK_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    // PortForwarder.add(5800, "photonvision.local", 5800);

    configureButtonBindings();
    /**
     * if(JOYSTICK.getRawButton(JoystickConstants.ARCADE_DRIVE_INVERT_BTN)){
     * m_driveTrain.setDefaultCommand(new ArcadeDriveCmd(m_driveTrain,
     * () -> -JOYSTICK.getRawAxis(JoystickConstants.ARCADE_DRIVE_SPEED_AXIS),
     * () -> JOYSTICK.getRawAxis(JoystickConstants.ARCADE_DRIVE_TURN_AXIS)));
     * }else if(JOYSTICK.getRawButton(JoystickConstants.ARCADE_DRIVE_REVERSE_BTN)){
     * m_driveTrain.setDefaultCommand(new ArcadeDriveCmd(m_driveTrain,
     * () -> JOYSTICK.getRawAxis(JoystickConstants.ARCADE_DRIVE_SPEED_AXIS),
     * () -> -JOYSTICK.getRawAxis(JoystickConstants.ARCADE_DRIVE_TURN_AXIS)));
     * }
     */

    m_driveTrain.setDefaultCommand(new ArcadeDriveCmd(m_driveTrain,() -> -JOYSTICK.getRawAxis(JoystickConstants.ARCADE_DRIVE_SPEED_AXIS),() -> JOYSTICK.getRawAxis(JoystickConstants.ARCADE_DRIVE_TURN_AXIS)));

    m_intake.setDefaultCommand(new IntakeCmd(m_intake, 0));
    m_indexer.setDefaultCommand(new IndexerCmd(m_indexer, 0));
    m_trigger.setDefaultCommand(new TriggerCmd(m_trigger, 0));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(JOYSTICK, 6).whileHeld(new IntakeCmd(m_intake, 1));
    new JoystickButton(JOYSTICK, 5).whileHeld(new VisionShooterCmd(m_shooter, m_hood, m_vision));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
