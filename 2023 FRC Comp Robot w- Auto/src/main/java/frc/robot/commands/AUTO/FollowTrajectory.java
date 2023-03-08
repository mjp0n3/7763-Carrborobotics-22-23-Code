// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AUTO;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;


public class FollowTrajectory extends CommandBase {

  private DrivetrainSubsystem drivetrainSubsystem;
  private String pathName;
  private boolean zeroInitialPose;

  /** Creates a new FollowTrajectory. */
  /** Creates a new FollowTrajectoryPathPlanner. */
  public FollowTrajectory(DrivetrainSubsystem drivetrainSubsystem, String pathName, boolean zeroInitialPose) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    this.pathName = pathName;
    this.zeroInitialPose = zeroInitialPose;
  
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10);
        
    new TrajectoryConfig(
          PathPlannerConstants.autoMaxVelocity,
          PathPlannerConstants.autoMaxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);
      // Makes a trajectory                                                     
      PathPlannerTrajectory trajectoryToFollow = PathPlanner.loadPath(pathName, PathPlannerConstants.autoMaxVelocity, PathPlannerConstants.autoMaxAcceleration);
    // Resets the pose of the robot if true (should generally only be true for the first path of an auto)
    if (zeroInitialPose) {
    drivetrainSubsystem.resetOdometry(trajectoryToFollow.getInitialPose() 
    );
    
    }

      

            // Create config for trajectory
    // TrajectoryConfig config =
    // new TrajectoryConfig(
    //       PathPlannerConstants.autoMaxVelocity,
    //       PathPlannerConstants.autoMaxAcceleration)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics)
    //     // Apply the voltage constraint
    //     .addConstraint(autoVoltageConstraint);

        RamseteCommand ramseteCommand = 
        new RamseteCommand(trajectoryToFollow, drivetrainSubsystem::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, drivetrainSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0), drivetrainSubsystem::tankDriveVolts,
        drivetrainSubsystem);

    // Create a PPSwerveControllerCommand. This is almost identical to WPILib's SwerveControllerCommand, but it uses the holonomic rotation from the PathPlannerTrajectory to control the robot's rotation.
    // -----idk if we need this this may be only for swerve but i think we need something like this but for tank drive-----
    // followTrajectoryPathPlannerCommand = new PPSwerveControllerCommand(
    //   trajectoryToFollow,
    //   drivetrainSubsystem::getPose, // Functional interface to feed supplier
    //   DriveConstants.kDriveKinematics,
    //   xController,
    //   yController,
    //   thetaController,
    //   drivetrainSubsystem::setModuleStates,
    //   drivetrainSubsystem
    // );
    
    // followTrajectoryPathPlannerCommand.schedule();
    
  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   done = followTrajectoryPathPlannerCommand.isFinished();
  // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }

  // Run path following command, then stop at the end.
  ramseteCommand.andThen(() -> drivetrainSubsystem.tankDriveVolts(0, 0));
  return;

}
}