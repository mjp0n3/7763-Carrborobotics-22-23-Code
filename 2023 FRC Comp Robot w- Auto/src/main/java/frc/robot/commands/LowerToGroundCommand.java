// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class LowerToGroundCommand extends CommandBase {
   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;

  /** Creates a new LowerToGround. */
  public LowerToGroundCommand(ArmSubsystem armsubsystem) {
   this.armSubsystem = armsubsystem;
      //addrequirements
   
      addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  armSubsystem.setArmCoast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  armSubsystem.setArmDown();
  



  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setmotorsoff();
    armSubsystem.setArmBreak();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
