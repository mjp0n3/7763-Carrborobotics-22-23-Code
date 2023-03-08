// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GrabberGrabCommand;
import frc.robot.commands.GrabberReleaseCommand;
import frc.robot.commands.LowerToGroundCommand;
import frc.robot.commands.RaiseToHighCommand;
import frc.robot.commands.AUTO.FS_2CubeAutoCommand;
import frc.robot.commands.AUTO.OS_CClimbAutoCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  //grabber subsytem 
  private final GrabberSubsystem grabbersubsystem = new GrabberSubsystem();

  //grabber grab command
  private final GrabberGrabCommand grabbergrabCommand = new GrabberGrabCommand(grabbersubsystem);
  //grabber release command
  private final GrabberReleaseCommand grabberreleaseCommand = new GrabberReleaseCommand(grabbersubsystem);
  //armsubsystem
  private final ArmSubsystem armsubsystem = new ArmSubsystem();
  //arm down command
  private final LowerToGroundCommand lowertogroundCommand = new LowerToGroundCommand(armsubsystem);
  //arm up command
  private final RaiseToHighCommand raisetohighCommand = new RaiseToHighCommand(armsubsystem);
  //fs 2cube auto command
  private final FS_2CubeAutoCommand fS_2CubeAutoCommand = new FS_2CubeAutoCommand(drivetrainsubsystem, armsubsystem, grabbersubsystem);
  //os 1 cube climb auto command
  private final OS_CClimbAutoCommand  oS_CClimbAutoCommand = new OS_CClimbAutoCommand(drivetrainsubsystem, armsubsystem, grabbersubsystem);

  // final Command oS_CClimbAutoCommand = new instantCommand(drivetrainsubsystem, armsubsystem, grabbersubsystem);
  // private final DriveCommand driveCommand = new DriveCommand(drivetrainsubsystem);

  public static Joystick joystick = new Joystick (Constants.JoystickAxis1);
  JoystickButton abutton = new JoystickButton(joystick, Constants.ButtonA);
  JoystickButton xbutton = new JoystickButton(joystick, Constants.ButtonX);
  JoystickButton zbutton = new JoystickButton(joystick, Constants.ButtonZ);
  JoystickButton bbutton = new JoystickButton(joystick, Constants.ButtonB);
  // JoystickButton ybutton = new JoystickButton(joystick, Constants.ButtonY);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */


  //sendable chooser
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

  
    // Configure the button bindings
    configureButtonBindings();
    drivetrainsubsystem.setDefaultCommand(driveCommand);
    //buttons
    xbutton.whileTrue(lowertogroundCommand);
    abutton.whileTrue(raisetohighCommand);
    // zbutton.whileTrue(lowertogroundCommand);
    // bbutton.whileTrue(raisetohighCommand);
    // ybutton.whileHeld(armCommand);
    
    //uncomment for the auto selector 


    // autoChooser.setDefaultOption(only score);
    autoChooser.addOption("fs 2 cube auto", fS_2CubeAutoCommand);
    autoChooser.addOption("os 1 cube+pickup+climb auto", oS_CClimbAutoCommand);

    //shuffleboard autonomous chooser
    Shuffleboard.getTab("Autonomous").add(autoChooser);

  }

  public Command loadPathPlannerTrajectoryToRamseteCommand(String filename, boolean resetOdometry)  {
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch(IOException exception){
      DriverStation.reportError("Unable to open trajectory" + filename,  exception.getStackTrace());
      System.out.println("unable to read from file" + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drivetrainsubsystem::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, drivetrainsubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0), drivetrainsubsystem::tankDriveVolts,
        drivetrainsubsystem);

        if (resetOdometry){
          return new SequentialCommandGroup(
            new InstantCommand(()->drivetrainsubsystem.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
        } else {
          return ramseteCommand;
        }

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
 
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
  
}
