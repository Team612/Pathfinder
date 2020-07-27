/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int TALON_FR_DRIVE = 2;
    public static int TALON_FL_DRIVE = 12;
    public static int TALON_BR_DRIVE = 3;
    public static int TALON_BL_DRIVE = 1;
    public static int ULTRASONIC_DRIVE_ONE = 2;
    public static int ULTRASONIC_DRIVE_TWO = 4;


    // Arbritrary values, will change later
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(10);

    public static final double kPDriveVel = 8.5;
    
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;


    public static final double kEncoderDistancePerPulse =(kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;


}
