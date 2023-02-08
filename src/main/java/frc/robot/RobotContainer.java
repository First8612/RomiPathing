// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XboxController m_controller = new XboxController(0);
  private final Field2d m_field = new Field2d();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    SmartDashboard.putData("Field", m_field);

    loadTrajectories(m_chooser);
    SmartDashboard.putData("Auton", m_chooser);

    configureButtonBindings();
  }

  public void periodic() {
    m_field.setRobotPose(m_drivetrain.getPose());
  }

  private void configureButtonBindings() {
  }

  private void loadTrajectories(SendableChooser<Command> chooser) {
    var path1Trajectory = PathPlanner.loadPath("Path1", new PathConstraints(4, 3));
    var trajectory1Command = new FollowTrajectoryCommand(m_drivetrain, path1Trajectory);
    chooser.addOption("Path 1", trajectory1Command);

    var path2Trajectory = PathPlanner.loadPath("Path2", new PathConstraints(4, 3));
    var trajectory2Command = new FollowTrajectoryCommand(m_drivetrain, path2Trajectory);
    chooser.addOption("Path 2", trajectory2Command);

    TrajectoryConfig config = new TrajectoryConfig(
        Units.feetToMeters(2.0), Units.feetToMeters(2.0));
    config.setKinematics(m_drivetrain.getKinematics());

    var hundredInches = Units.inchesToMeters(100);

    var simpleTrajectory = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(
            new Pose2d(), new Pose2d(hundredInches, 0, new Rotation2d())
        // , new Pose2d(hundredInches, hundredInches, new Rotation2d(180))
        // , new Pose2d(hundredInches, -hundredInches, new Rotation2d())
        // , new Pose2d(hundredInches, 0, new Rotation2d())
        // , new Pose2d(0,0, new Rotation2d())
        ),
        config);
    var simpleTrajectoryCommand = new FollowTrajectoryCommand(m_drivetrain, simpleTrajectory);
    chooser.addOption("Path 3", simpleTrajectoryCommand);
    
    m_chooser.setDefaultOption("Path 1", trajectory1Command);
  }

  public Command getAutonomousCommand() {
    m_drivetrain.reset();

    return m_chooser.getSelected();
  }

  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getRawAxis(1), () -> -m_controller.getRawAxis(2));
  }
}
