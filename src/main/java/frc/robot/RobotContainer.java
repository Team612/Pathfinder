/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain m_robotDrive=new DriveTrain();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private Trajectory trajectory;



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }


  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,Constants.kvVoltSecondsPerMeter,Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

            TrajectoryConfig config =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,Constants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.kDriveKinematics)
                 .addConstraint(autoVoltageConstraint);
                 /*
    // An ExampleCommand will run in autonomous
    String trajectoryJSON = "paths/YourPath.wpilib.json";
try {
  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex) {
  DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
}
    return m_autoCommand;

  }*/

  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(

        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        new Pose2d(3, 0, new Rotation2d(0)),
        config
    );

  RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_robotDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,Constants.kvVoltSecondsPerMeter,
        Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    );

}

