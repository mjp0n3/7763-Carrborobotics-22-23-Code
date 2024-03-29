// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

//unused imports hashed out
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatusFrame;
// import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// import edu.wpi.first.wpilibj.I2C.Port;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class GrabberSubsystem extends SubsystemBase {
  /** Creates a new GrabberSubsystem. */
  // private final DoubleSolenoid GrabberSolenoid;

  //intake motors
  WPI_TalonSRX IntakeMotor = new WPI_TalonSRX(Constants.GrabberConstants.RightIntakeMotor);
  
  
  
  public GrabberSubsystem() {
    //grabber solenoids
    //  GrabberSolenoid = new DoubleSolenoid(ElectronicsConstants.kPneumaticsModuleType,
    // GrabberConstants.GrabberDeployedPort, GrabberConstants.GrabberRetractedPort);
  
    }
  
  //solenoid retract intake cone
  // public void IntakeCone() {
  //   GrabberSolenoid.set(Value.kReverse);
  // }
  // //might need to reverse these
  // //solenoid deploy outake cone 
  // public void OutakeCone() {
  //   GrabberSolenoid.set(Value.kForward);
  // }

  //intake cube
  public void IntakeCube() {
    IntakeMotor.set(.6);
  }
  
  //outake cube
  public void OutakeCube() {
    IntakeMotor.set(-1);
   
  }
   //intake stop 
   public void Intakeoff() {
    IntakeMotor.set(0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
