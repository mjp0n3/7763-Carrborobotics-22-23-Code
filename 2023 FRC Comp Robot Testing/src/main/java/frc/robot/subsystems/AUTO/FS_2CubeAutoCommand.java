// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AUTO;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.commands.GrabberGrabCommand;
import frc.robot.commands.GrabberReleaseCommand;
import frc.robot.commands.LowerToGroundCommand;
import frc.robot.commands.RaiseToHighCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FS_2CubeAutoCommand extends ParallelCommandGroup {
  /** Creates a new FS_2CubeAutoCommand. */
  public FS_2CubeAutoCommand(ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem,
  DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //  --all steps for the auto go in here--

    //  1. Backs up 
    new ParallelCommandGroup(
            new FollowTrajectory(drivetrainSubsystem, "FS_2Cube1", true)
  ).withTimeout(2),


    //  2. Raises arm
    new ParallelCommandGroup(
      new RaiseToHighCommand(armSubsystem)
  ).withTimeout(2),

    //  3. Drives forwards 
    new ParallelCommandGroup(
      new FollowTrajectory(drivetrainSubsystem, PathPlannerConstants.fs_2cubepath2, false)
  ).withTimeout(0.5),


    //  4. Scores first cube
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
  ).withTimeout(0.5),

    //  5. Drives backwards and spins around
    new ParallelCommandGroup(
      new FollowTrajectory(drivetrainSubsystem, PathPlannerConstants.fs_2cubepath3, false)
  ).withTimeout(0.5),


    //  6. Arm goes down
    new ParallelCommandGroup(
      new LowerToGroundCommand(armSubsystem)
  ).withTimeout(0.5),
    
    //  7. Drives to the second cube and picks it up
    new ParallelCommandGroup(
      new FollowTrajectory(drivetrainSubsystem, PathPlannerConstants.fs_2cubepath4, false),
      new GrabberGrabCommand(grabberSubsystem)
      
  ).withTimeout(0.5),


    //  8. Spins around and drives back to community
    new ParallelCommandGroup(
          new FollowTrajectory(drivetrainSubsystem,  PathPlannerConstants.fs_2cubepath2, false)
  ).withTimeout(0.5),


    //  9. Raises arm
    new ParallelCommandGroup(
      new RaiseToHighCommand(armSubsystem)
  ).withTimeout(0.5),
    

    //  10. Scores 2nd cube
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
 ).withTimeout(0.5)


    //end
        
        


    );
  }
}
