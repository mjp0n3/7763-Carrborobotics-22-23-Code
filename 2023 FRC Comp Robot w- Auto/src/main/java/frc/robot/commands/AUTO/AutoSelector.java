// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AUTO;

import javax.management.InstanceAlreadyExistsException;

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
  
  // 3. score 1 cube and balance (middle) (!Not tested!, tune the 360 turn )

  // 4. balance only (middle) (should be good?)


autochooser.addOption("score and balance (360 timed)", new SequentialCommandGroup(

   //1. arm up
   new ParallelCommandGroup(
    new RaiseToHighCommand(armSubsystem)
  ).withTimeout(2.0),


    //2. outake
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
  ).withTimeout(1),

    //2. 360+arm down
    new ParallelCommandGroup(
      new InstantCommand( ()->drivetrainSubsystem.arcadeDrive(0.35, 0)),
      new LowerToGroundCommand(armSubsystem)
  ).withTimeout(2),

    
    // 3. drive to balance
      new AutoDriveToPlatformCommand(drivetrainSubsystem,  0.7, 8, 0.3), //ajust these maybe? idk

    //4. balance
      new AutoBalanceCommand(drivetrainSubsystem)
    //end of score and balance
    ));


autochooser.addOption("balance only", new SequentialCommandGroup(
  //  . drive forward to charge station

  new AutoDriveToPlatformCommand(drivetrainSubsystem, 0.7, 8, 0.3), //ajust these maybe? idk

  
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
  ).withTimeout(3)
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
  ).withTimeout(1),

// autochooser.addOption("old score and balance", new SequentialCommandGroup(

// //1. raise arm
// new InstantCommand(() -> armSubsystem.setArmHighCube()
// ).withTimeout(1.5),


// //2. drive forward
// new InstantCommand(() -> drivetrainSubsystem.arcadeDrive(0, 0.3)  //might need to ajust the going forward bit idk, possibly reverse it? idk
// ).withTimeout(0.5),

// //3. outake cube
// new InstantCommand(() -> grabberSubsystem.OutakeCube()
// ).withTimeout(0.5),

// //4. stop outake cube
// new InstantCommand(() -> grabberSubsystem.Intakeoff()
// ).withTimeout(0.2)

// ));

// 5. drive to balance
new AutoDriveToPlatformCommand(drivetrainSubsystem, 0.7, 8, 0.075), //ajust these maybe? idk

//6. balance
new AutoBalanceCommand(drivetrainSubsystem)
));

autochooser.addOption("old FS_2CubeAUTOCOMMAND", new SequentialCommandGroup(

    //  1. Raises arm
    new ParallelCommandGroup(
      new RaiseToHighCommand(armSubsystem)
  ).withTimeout(0.5),

    //  2. Drives forwards 
    new ParallelCommandGroup(
      drivetrainSubsystem.followTrajectoryCommand("FS_2Cube1",false, 1, 1)
  ).withTimeout(1.68),


    //  3. Scores first cube
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
  ).withTimeout(0.5),

    //  4. Drives backwards 
    new ParallelCommandGroup(
      drivetrainSubsystem.followTrajectoryCommand("FS_2Cube2",false, 1, 1)
  ).withTimeout(2.17),


    //  5. Arm goes down
    new ParallelCommandGroup(
      new LowerToGroundCommand(armSubsystem)
  ).withTimeout(0.5),
    
    //  6. Drives to the second cube 
    new ParallelCommandGroup(
        drivetrainSubsystem.followTrajectoryCommand("FS_2Cube3",false, 1, 1),
      new GrabberGrabCommand(grabberSubsystem)
  ).withTimeout(2.84),

  //  7. Drives forward and picks up cube
  new ParallelCommandGroup(
    drivetrainSubsystem.followTrajectoryCommand("FS_2Cube4",false, 1, 1),
  new GrabberGrabCommand(grabberSubsystem)
).withTimeout(1.41),

    //  8. Spins around and drives back to community
    new ParallelCommandGroup(
      drivetrainSubsystem.followTrajectoryCommand("FS_2Cube5",false, 1, 1)
  ).withTimeout(3.57),


    //  9. Raises arm
    new ParallelCommandGroup(
      new RaiseToHighCommand(armSubsystem)
  ).withTimeout(0.5),

    // 10. Drives forward
    new ParallelCommandGroup(
      drivetrainSubsystem.followTrajectoryCommand("FS_2Cube6", false, 1, 1)
  ).withTimeout(1.54),

    //  12. Scores 2nd cube
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
 ).withTimeout(0.5)
  //end of FS2CubeAutoCommand
  ));

SmartDashboard.putData("Auto Selector", autochooser);
  }
  public Command getSelected() {
    return autochooser.getSelected();
  }
}