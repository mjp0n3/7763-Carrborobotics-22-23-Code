// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AUTO;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.GrabberGrabCommand;
import frc.robot.commands.GrabberReleaseCommand;
import frc.robot.commands.LowerToGroundCommand;
import frc.robot.commands.RaiseToHighCommand;
public class AutoSelector {
  /** Creates a new AutoSelector. */
  private final SendableChooser<Command> autochooser = new SendableChooser<>();

  private final Timer timer = new Timer();
  
  public AutoSelector(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
    //currently we have a 
    //-2 cube auto (left/feeder side)
    //-score one then balance (center)
    //-score only (any side )
    //later add the other side one for a 2 cube there
    //a testing one that just prints that it ran

    autochooser.addOption("FS_2CubeAUTOCOMMAND", new SequentialCommandGroup(

   


    //  1. Raises arm
    new ParallelCommandGroup(
      new RaiseToHighCommand(armSubsystem)
  ).withTimeout(2),

    //  2. Drives forwards 
    new ParallelCommandGroup(
      drivetrainSubsystem.followTrajectoryCommand("FS_2Cube1", false)
  ).withTimeout(0.5),


    //  3. Scores first cube
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
  ).withTimeout(0.5),

    //  4. Drives backwards 
    new ParallelCommandGroup(
      drivetrainSubsystem.followTrajectoryCommand("FS_2Cube2", false)
  ).withTimeout(0.5),


    //  5. Arm goes down
    new ParallelCommandGroup(
      new LowerToGroundCommand(armSubsystem)
  ).withTimeout(0.5),
    
    //  6. Drives to the second cube 
    new ParallelCommandGroup(
        drivetrainSubsystem.followTrajectoryCommand("FS_2Cube3", false),
      new GrabberGrabCommand(grabberSubsystem)
  ).withTimeout(0.5),

  //  7. Drives forward and picks up cube
  new ParallelCommandGroup(
    drivetrainSubsystem.followTrajectoryCommand("FS_2Cube4", false),
  new GrabberGrabCommand(grabberSubsystem)
).withTimeout(0.5),

    //  8. Spins around and drives back to community
    new ParallelCommandGroup(
      drivetrainSubsystem.followTrajectoryCommand("FS_2Cube5", false)
  ).withTimeout(0.5),


    //  9. Raises arm
    new ParallelCommandGroup(
      new RaiseToHighCommand(armSubsystem)
  ).withTimeout(0.5),

    // 10. Drives forward
    new ParallelCommandGroup(
      drivetrainSubsystem.followTrajectoryCommand("FS_2Cube6", false)
  ).withTimeout(0.5),

    //  12. Scores 2nd cube
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
 ).withTimeout(0.5)

  //end of FS2CubeAutoCommand
    
  ));


    
  autochooser.addOption("option2", new SequentialCommandGroup(
   
      new PrintCommand("option 2 ran")

    //end of option 2 auto
      
  ));

  autochooser.addOption("Score 1 Cube only", new SequentialCommandGroup(

    //1. raise arm
    new InstantCommand(() -> armSubsystem.setArmHighCube()
  ).withTimeout(0.5),

    
    //2. drive forward
    new ParallelCommandGroup(
      drivetrainSubsystem.followTrajectoryCommand("MiddleForward", false)
  ).withTimeout(0.5),

    //3. outake cube
    new InstantCommand(() -> grabberSubsystem.OutakeCube()
  ).withTimeout(0.5)

    //end of single score auto
      
));

  autochooser.addOption("score and balance", new SequentialCommandGroup(

    //1. raise arm
    new InstantCommand(() -> armSubsystem.setArmHighCube()
  ).withTimeout(0.5),

    
    //2. drive forward
    new ParallelCommandGroup(
      drivetrainSubsystem.followTrajectoryCommand("MiddleForward", false)
  ).withTimeout(0.5),

    //3. outake cube
    new InstantCommand(() -> grabberSubsystem.OutakeCube()
  ).withTimeout(0.5),
  //end of score and balance auto

  //4. drive to balance
    new AutoDriveToPlatformCommand(drivetrainSubsystem, 0.5, 15, 2),

  //5. balance
    new AutoBalanceCommand(drivetrainSubsystem)

));

  }
  public Command getSelected() {
    return autochooser.getSelected();
  }
}