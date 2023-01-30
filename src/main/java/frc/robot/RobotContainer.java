// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RobotContainer {
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XboxController m_controller = new XboxController(0);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    m_drivetrain.reset();

    TrajectoryConfig config = new TrajectoryConfig(
        Units.feetToMeters(2.0), Units.feetToMeters(2.0));
    config.setKinematics(m_drivetrain.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d()),
            new Pose2d(2.3, 1.2, Rotation2d.fromDegrees(90.0))),
        config
    );

    RamseteCommand command = new RamseteCommand(
        trajectory,
        m_drivetrain::getPose,
        new RamseteController(2, .7),
        m_drivetrain.getFeedforward(),
        m_drivetrain.getKinematics(),
        m_drivetrain::getSpeeds,
        m_drivetrain.getLeftPIDController(),
        m_drivetrain.getRightPIDController(),
        m_drivetrain::setOutputVolts,
        m_drivetrain
    );

    return command.andThen(() -> m_drivetrain.setOutputVolts(0, 0));
  }

  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getRawAxis(1), () -> -m_controller.getRawAxis(2));
  }
}
