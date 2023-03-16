// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AUTO;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FS_2CubeAutoCommand extends SequentialCommandGroup {
  /** Creates a new FS_2CubeAutoCommand. */
  public FS_2CubeAutoCommand(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
    
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(
    //  --all steps for the auto go in here--  \\ 

//     //  1. Backs up 
//     new ParallelCommandGroup(
//         // old using command: new FollowTrajectory(drivetrainSubsystem, PathPlannerConstants.fs_2cubepath1, true)         
//         drivetrainSubsystem.followTrajectoryCommand(PathPlannerConstants.fs_2cubepath1, true)
//   ).withTimeout(2),


//     //  2. Raises arm
//     new ParallelCommandGroup(
//       new RaiseToHighCommand(armSubsystem)
//   ).withTimeout(2),

//     //  3. Drives forwards 
//     new ParallelCommandGroup(
//       drivetrainSubsystem.followTrajectoryCommand(PathPlannerConstants.fs_2cubepath2, false)
//   ).withTimeout(0.5),


//     //  4. Scores first cube
//     new ParallelCommandGroup(
//       new GrabberReleaseCommand(grabberSubsystem)
//   ).withTimeout(0.5),

//     //  5. Drives backwards and spins around
//     new ParallelCommandGroup(
//       drivetrainSubsystem.followTrajectoryCommand(PathPlannerConstants.fs_2cubepath3, false)
//   ).withTimeout(0.5),


//     //  6. Arm goes down
//     new ParallelCommandGroup(
//       new LowerToGroundCommand(armSubsystem)
//   ).withTimeout(0.5),
    
//     //  7. Drives to the second cube and picks it up
//     new ParallelCommandGroup(
//         drivetrainSubsystem.followTrajectoryCommand(PathPlannerConstants.fs_2cubepath4, false),
//       new GrabberGrabCommand(grabberSubsystem)
//   ).withTimeout(0.5),


//     //  8. Spins around and drives back to community
//     new ParallelCommandGroup(
//       drivetrainSubsystem.followTrajectoryCommand(PathPlannerConstants.fs_2cubepath5, false)
//   ).withTimeout(0.5),


//     //  9. Raises arm
//     new ParallelCommandGroup(
//       new RaiseToHighCommand(armSubsystem)
//   ).withTimeout(0.5),
    

//     //  10. Scores 2nd cube
//     new ParallelCommandGroup(
//       new GrabberReleaseCommand(grabberSubsystem)
//  ).withTimeout(0.5)


//   //end
    
//   );
    
  }
}
