// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AUTO;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  ).withTimeout(1),


    //2. outake
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
  ).withTimeout(1),
    
    //3. intake stop
    new RunCommand( ()->grabberSubsystem.Intakeoff(), grabberSubsystem).withTimeout(0.1),

    //4. lower arm
    new LowerToGroundCommand(armSubsystem).withTimeout(1.65),

    //5. drive over charge station for mobility points
    // new RunCommand( ()->drivetrainSubsystem.curvatureDrive(0,0.75), drivetrainSubsystem).withTimeout(2),

    //6. drive to balance
    new AutoDriveToPlatformCommand(drivetrainSubsystem,  -0.7, 8, 0.075).withTimeout(6), //ajust these maybe? idk

    //7. balance
    new AutoBalanceCommand(drivetrainSubsystem)
    //end of score and balance
    ));



    
autochooser.addOption("balance only", new SequentialCommandGroup(
  //  . drive forward to charge station

  new AutoDriveToPlatformCommand(drivetrainSubsystem, -0.7, 8, 0.075).withTimeout(6), //ajust these maybe? idk

  //2. balance!

  new AutoBalanceCommand(drivetrainSubsystem)

));

autochooser.addOption("drive only", 
  //  . drive forward to charge station

  new RunCommand( ()->drivetrainSubsystem.tankDriveVolts(6, 6), drivetrainSubsystem)

);

autochooser.addOption("score and dip", new SequentialCommandGroup(
    //arm up
    new ParallelCommandGroup(
    new RaiseToHighCommand(armSubsystem)
  ).withTimeout(1),


    //2. outake
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
  ).withTimeout(1),

    //3. intake stop
    new RunCommand( ()->grabberSubsystem.Intakeoff(), grabberSubsystem).withTimeout(0.1),

    new ParallelCommandGroup(
      new RunCommand( ()->drivetrainSubsystem.curvatureDrive(0,0.7), drivetrainSubsystem),
      new LowerToGroundCommand(armSubsystem)
  ).withTimeout(1.5)
    //end of score and dip new
));

autochooser.addOption("score ONLY", new SequentialCommandGroup(
    //arm up
    new ParallelCommandGroup(
      new RaiseToHighCommand(armSubsystem)
  ).withTimeout(1.2),


    //2. outake
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
  ).withTimeout(1),

   //3. intake stop
    new RunCommand( ()->grabberSubsystem.Intakeoff(), grabberSubsystem).withTimeout(0.1),

    //arm down
    new LowerToGroundCommand(armSubsystem)
  .withTimeout(1.5)

));
autochooser.setDefaultOption("pathplanner test", new SequentialCommandGroup(


    //  1. Drives forwards 
    
      drivetrainSubsystem.followTrajectoryCommand("TestPath",true, 0.01, 0.01)
  

  ));
  //Feeder Side 2 Cube Auto
  autochooser.addOption("Feeder Side 2 Cube Auto ", new SequentialCommandGroup(

    //score one

    //  1. arm up
   new ParallelCommandGroup(
    new RaiseToHighCommand(armSubsystem)
  ).withTimeout(0.8),


    //  2. outake
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
  ).withTimeout(1),
    
    //  3. intake stop
    new RunCommand( ()->grabberSubsystem.Intakeoff(), grabberSubsystem).withTimeout(0.1),

    //  4. lower arm
    new LowerToGroundCommand(armSubsystem).withTimeout(1.65),

    //  5. Drive to 2nd cube
      new RunCommand( ()->
        drivetrainSubsystem.followTrajectoryCommand("FS2_Path1",true, 2, 3)
      ).withTimeout(1),
    
    //  6.Intake and Drive 
    new ParallelCommandGroup(
      new GrabberGrabCommand(grabberSubsystem),
      drivetrainSubsystem.followTrajectoryCommand("FS2_Path2",false, 2, 3)
    ).withTimeout(1),

    //  7. Drive back 
    new RunCommand( ()->
      drivetrainSubsystem.followTrajectoryCommand("FS2_Path3",false, 2, 3)
    ).withTimeout(1),

    //  8. arm up
   new ParallelCommandGroup(
    new RaiseToHighCommand(armSubsystem)
  ).withTimeout(2.0),


    //  9. outake
    new ParallelCommandGroup(
      new GrabberReleaseCommand(grabberSubsystem)
  ).withTimeout(1),
    
    //  10. intake stop
    new RunCommand( ()->grabberSubsystem.Intakeoff(), grabberSubsystem)
    .withTimeout(0.1),

    //  11. lower arm
    new LowerToGroundCommand(armSubsystem)
    .withTimeout(1.65),

    //  12. Drive to 3rd cube
    new RunCommand( ()->
      drivetrainSubsystem.followTrajectoryCommand("FS2_Path4",false, 2, 3)
    ).withTimeout(1)

    //  end of 2 cube auto
  ));



SmartDashboard.putData("Auto Selector", autochooser);
  }
  public Command getSelected() {
    return autochooser.getSelected();
  }
}