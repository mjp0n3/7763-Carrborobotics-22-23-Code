// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AUTO;

import javax.management.InstanceAlreadyExistsException;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.GrabberGrabCommand;
import frc.robot.commands.GrabberReleaseCommand;
import frc.robot.commands.LowerToGroundCommand;
import frc.robot.commands.RaiseToHighCommand;
public class AutoSelector {
  /** Creates a new AutoSelector. */
  private final SendableChooser<Command> autochooser = new SendableChooser<>();


  private final Timer timer = new Timer();
  
  public AutoSelector(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {

  //autos:
  // 1. score 1 cube high only (anywhere) (DONE 'n TESTED)
  
  // 2. score 1 cube and mobility (sides) (Mobility is iffy)
  
  // 3. score 1 cube and balance (middle) (!Not tested!, tune balance )

  // 4. balance only (middle) (should be good?)


autochooser.addOption("score and balance (backwards climb)", new SequentialCommandGroup(

   //1. arm up
   new ParallelCommandGroup(
    new RaiseToHighCommand(armSubsystem)
  ).withTimeout(2.0),


    //2. outake
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
  ).withTimeout(1),
    
    // 3. drive to balance
      new AutoDriveToPlatformCommand(drivetrainSubsystem,  -0.7, 8, 0.075).withTimeout(6), //ajust these maybe? idk

    //4. balance
      new AutoBalanceCommand(drivetrainSubsystem)
    //end of score and balance
    ));


autochooser.addOption("balance only", new SequentialCommandGroup(
  //  . drive forward to charge station

  new AutoDriveToPlatformCommand(drivetrainSubsystem, 0.7, 8, 0.075).withTimeout(6), //ajust these maybe? idk

  //2. balance!

  new AutoBalanceCommand(drivetrainSubsystem)

));

autochooser.addOption("drive only", 
  //  . drive forward to charge station

  new RunCommand( ()->drivetrainSubsystem.tankDriveVolts(6, 6), drivetrainSubsystem)

);

autochooser.setDefaultOption("score and dip", new SequentialCommandGroup(
    //arm up
    new ParallelCommandGroup(
    new RaiseToHighCommand(armSubsystem)
  ).withTimeout(2.0),


    //2. outake
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
  ).withTimeout(1),


    new ParallelCommandGroup(
      new RunCommand( ()->drivetrainSubsystem.arcadeDrive(0,0.7), drivetrainSubsystem),
      new LowerToGroundCommand(armSubsystem)
  ).withTimeout(2)
    //end of score and dip new
));

autochooser.addOption("score ONLY", new SequentialCommandGroup(
    //arm up
    new ParallelCommandGroup(
      new RaiseToHighCommand(armSubsystem)
  ).withTimeout(2.0),


    //2. outake
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
  ).withTimeout(1)

));
autochooser.addOption("pathplanner test", new SequentialCommandGroup(


    //  2. Drives forwards 
    new ParallelCommandGroup(
      drivetrainSubsystem.followTrajectoryCommand("New Test",true, 1, 1)

      ).withTimeout(2.67)

  ));

SmartDashboard.putData("Auto Selector", autochooser);
  }
  public Command getSelected() {
    return autochooser.getSelected();
  }
}