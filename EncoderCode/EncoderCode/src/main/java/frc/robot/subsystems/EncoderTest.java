/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Encoder;

public class EncoderTest extends SubsystemBase {
  /**
   * Creates a new EncoderTest.
   */
  public EncoderTest() {
    
  }

  final WPI_TalonSRX talon_fr_drive = new WPI_TalonSRX(Constants.TALON_FR_DRIVE);
  final WPI_TalonSRX talon_fl_drive = new WPI_TalonSRX(Constants.TALON_FL_DRIVE);
  final WPI_TalonSRX talon_br_drive = new WPI_TalonSRX(Constants.TALON_BR_DRIVE);
  final WPI_TalonSRX talon_bl_drive = new WPI_TalonSRX(Constants.TALON_BL_DRIVE);
  
  public void tankDrive(double left_command, double right_command){
    left_command = Math.abs(left_command);
    right_command = Math.abs(right_command);

    talon_fr_drive.set(-right_command);
    talon_br_drive.set(-right_command);

    talon_fl_drive.set(left_command);
    talon_bl_drive.set(left_command);

  }

  public Encoder encoder0 = new Encoder(Constants.ENCODER_PORT_ONE, Constants.ENCODER_PORT_TWO);
  public Encoder encoder1 = new Encoder(Constants.ENCODER_PORT_THREE, Constants.ENCODER_PORT_FOUR);
  public Encoder encoder2 = new Encoder(Constants.ENCODER_PORT_FIVE, Constants.ENCODER_PORT_SIX);
  public Encoder encoder3 = new Encoder(Constants.ENCODER_PORT_SEVEN, Constants.ENCODER_PORT_EIGHT);


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
