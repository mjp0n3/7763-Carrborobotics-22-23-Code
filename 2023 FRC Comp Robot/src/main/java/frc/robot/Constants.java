// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DrivetrainConstants{

        public static final int RightFrontID = 2;
        public static final int RightBackID = 3;
        public static final int LeftFrontID = 4;
        public static final int LeftBackID = 6;

    }

    public static final int JoystickAxis1 = 0;
    public static final int ButtonA = 1;
    public static final int ButtonB = 2;
    public static final int ButtonX = 3;
    public static final int ButtonY= 4;
    public static final int ButtonZ= 5;

    public static final class ElectronicsConstants {
        public static PneumaticsModuleType kPneumaticsModuleType = PneumaticsModuleType.CTREPCM;
        public static int kLEDPort = 0;
    }
    public static final class GrabberConstants {
        //solenoid ports
        public static final int GrabberDeployedPort = 2;
        public static final int GrabberRetractedPort = 3;
    }

}
