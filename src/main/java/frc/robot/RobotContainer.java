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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RobotContainer {
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XboxController m_controller = new XboxController(0);
  private final Field2d m_field = new Field2d();
  
  public RobotContainer() {
    SmartDashboard.putData("Field", m_field);
    
    configureButtonBindings();
  }

  public void periodic() {
    m_field.setRobotPose(m_drivetrain.getPose());
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    m_drivetrain.reset();

    TrajectoryConfig config = new TrajectoryConfig(
        Units.feetToMeters(2.0), Units.feetToMeters(2.0));
    config.setKinematics(m_drivetrain.getKinematics());

    var hundredInches = Units.inchesToMeters(100);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
          new Pose2d()
          , new Pose2d(hundredInches, 0, new Rotation2d())
          , new Pose2d(hundredInches, hundredInches, new Rotation2d(180))
          , new Pose2d(hundredInches, -hundredInches, new Rotation2d())
          // , new Pose2d(hundredInches, 0, new Rotation2d())
          // , new Pose2d(0,0, new Rotation2d())
        ),
        config
    );

    return new FollowTrajectoryCommand(m_drivetrain, trajectory);
  }

  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getRawAxis(1), () -> -m_controller.getRawAxis(2));
  }
}
