// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Thank you to Eastbots FRC 4795 for help with balance auto 

package frc.robot.commands.AUTO;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;



public class AutoDriveToPlatformCommand extends CommandBase{
    DrivetrainSubsystem drivetrainSubsystem;
    double speed;
    double angleThreshold;


    double duration;
    double time;
    boolean check;


    double elevationAngle;

//                                                                                    (=0.4)             (=8)                  (=0.075)
    public AutoDriveToPlatformCommand(DrivetrainSubsystem drivetrainSubsystem, double speed , double angleThreshold , double checkDuration) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.angleThreshold = angleThreshold;
        this.speed = -speed;
        this.duration = checkDuration;
        this.time = 0;
        this.check = false;
        addRequirements(drivetrainSubsystem);
    }


    @Override
    public void initialize(){
        elevationAngle = drivetrainSubsystem.getElevationAngle();
    }


    @Override
    public void execute(){
        drivetrainSubsystem.curvatureDrive(0, speed); //same as the previous comment^^
        elevationAngle = drivetrainSubsystem.getElevationAngle();
        if(Math.abs(elevationAngle) > angleThreshold){
            if(!check){
                time = Timer.getFPGATimestamp() + 100;
                check = true;
            }
        } else {
            check = false;
            time = 0;
        }
    }


    @Override
    public boolean isFinished(){
        return (((Timer.getFPGATimestamp() + 100 - time) > duration) && check);
    }
}
