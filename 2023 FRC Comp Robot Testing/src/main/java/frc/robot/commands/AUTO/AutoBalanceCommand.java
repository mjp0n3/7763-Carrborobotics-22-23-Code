// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Thank you to Eastbots FRC 4795 for help with balance auto 

package frc.robot.commands.AUTO;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Balanceconstants;
import frc.robot.subsystems.DrivetrainSubsystem;



public class AutoBalanceCommand extends CommandBase{
    DrivetrainSubsystem drivetrainSubsystem;


    double elevationAngle;
    double output;


    public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem){
        this.drivetrainSubsystem = drivetrainSubsystem;
        output = 0;
        addRequirements(drivetrainSubsystem);
    }


    @Override
    public void initialize(){
        elevationAngle = drivetrainSubsystem.getElevationAngle();
    }


    @Override
    public void execute(){
        elevationAngle = drivetrainSubsystem.getElevationAngle();
        output = updateDrive();         //rot   //fwd
        drivetrainSubsystem.curvatureDrive(0, output); //assuming tank drive defined as drive(speed, rotation), add extra arguments as necessary
    }


    private double updateDrive() {
        return -signOf(elevationAngle)*(Math.pow(Balanceconstants.angleCoefficient  * (Math.abs(elevationAngle) / Balanceconstants.platformMaxAngle), 2)) * Balanceconstants.balanceSpeed;

    }


    private double signOf(double num){
        if(num < 0){
            return -1; //was -.45
        } else if (num > 0){
            return 1; //was .45
        } else {
            return 0;
        }
    }


    @Override
    public boolean isFinished(){
        return false;
    }
}
