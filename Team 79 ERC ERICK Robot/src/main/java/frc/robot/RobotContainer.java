// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainsubsystem = new DrivetrainSubsystem();

  private final DriveCommand driveCommand = new DriveCommand(drivetrainsubsystem);

  public static Joystick joystick = new Joystick (Constants.JoystickAxis1);

  //Intake command and subsystem
  private final IntakeSubsystem intakesubsystem = new IntakeSubsystem();

  private final IntakeCommand intakeCommand = new IntakeCommand(intakesubsystem);

  //outtake command

  private final OuttakeCommand outtakeCommand = new OuttakeCommand(intakesubsystem);

  //Arm command and subsystem
  private final ArmSubsystem armsubsystem = new ArmSubsystem();

  private final ArmCommand armCommand = new ArmCommand(armsubsystem);
  
   //Arm down command 
   private final ArmDownCommand armdownCommand = new ArmDownCommand(armsubsystem);
 
  
  
 //create joystick buttons
  JoystickButton abutton = new JoystickButton(joystick, Constants.ButtonA);
  JoystickButton xbutton = new JoystickButton(joystick, Constants.ButtonX);
  JoystickButton bbutton = new JoystickButton(joystick, Constants.ButtonB);
  JoystickButton ybutton = new JoystickButton(joystick, Constants.ButtonY);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings including joysticks and buttons
    configureButtonBindings();
    drivetrainsubsystem.setDefaultCommand(driveCommand);
    //buttons
    xbutton.whileHeld(intakeCommand);
    abutton.whileHeld(outtakeCommand);
    ybutton.whileHeld(armCommand);
    bbutton.whileHeld(armdownCommand);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return driveCommand;
  }
  
}
