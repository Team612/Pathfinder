/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
 

  /**
   * Creates a new DriveTrain.
   */
  final double DEADZONE = 0.1;

  final WPI_TalonSRX talon_fr_drive = new WPI_TalonSRX(Constants.TALON_FR_DRIVE);
  final WPI_TalonSRX talon_fl_drive = new WPI_TalonSRX(Constants.TALON_FL_DRIVE);
  final WPI_TalonSRX talon_br_drive = new WPI_TalonSRX(Constants.TALON_BR_DRIVE);
  final WPI_TalonSRX talon_bl_drive = new WPI_TalonSRX(Constants.TALON_BL_DRIVE);
  
  private final Gyro m_gyro = new ADXRS450_Gyro();


  private final DifferentialDriveOdometry m_odometry;

  private final Encoder right_encoder=new Encoder(1,2,3);
  private final Encoder left_encoder=new Encoder(4,5,6);

  public void tankDrive(double left_command, double right_command) {
    left_command = Math.abs(left_command) < DEADZONE ? 0.0 : left_command;
    right_command = Math.abs(right_command) < DEADZONE ? 0.0 : right_command;

    talon_fr_drive.set(-right_command);
    talon_br_drive.set(-right_command);

    talon_fl_drive.set(left_command);
    talon_bl_drive.set(left_command);

  }
  public DriveTrain(){
    left_encoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    right_encoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left_encoder.getRate(), right_encoder.getRate());
  }
  public void tankDriveVolts(final double leftVolts, final double rightVolts) {
    talon_fl_drive.setVoltage(leftVolts);
    talon_bl_drive.setVoltage(leftVolts);

    talon_fr_drive.setVoltage(-rightVolts);
    talon_br_drive.setVoltage(-rightVolts);
    
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
