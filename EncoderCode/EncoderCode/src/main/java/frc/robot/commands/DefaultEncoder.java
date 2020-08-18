/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.ControlMap;
import frc.robot.subsystems.EncoderTest;

public class DefaultEncoder extends CommandBase {
  /**
   * Creates a new DefaultEncoder.
   */
  private final EncoderTest m_encoder_test;

  public DefaultEncoder(EncoderTest encoder) {
    m_encoder_test = encoder;
    addRequirements(encoder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_encoder_test.encoder0.reset();
    m_encoder_test.encoder1.reset();
    m_encoder_test.encoder2.reset();
    m_encoder_test.encoder3.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_encoder_test.encoder0.get());
    System.out.println(m_encoder_test.encoder1.get());
    System.out.println(m_encoder_test.encoder2.get());
    System.out.println(m_encoder_test.encoder3.get());
    m_encoder_test.tankDrive(ControlMap.driver.getY(Hand.kLeft),ControlMap.driver.getY(Hand.kRight));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
