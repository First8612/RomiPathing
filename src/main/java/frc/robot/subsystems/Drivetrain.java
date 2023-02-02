// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeters = 0.07;
  private static final double kChasisWidthMeters = Units.inchesToMeters(5.95);
  public static final double ksVolts = 0.929;
  public static final double kvVoltSecondsPerMeter = 6.33;
  public static final double kaVoltSecondsSquaredPerMeter = 0.0389;

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kChasisWidthMeters);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

  double p = 0.085, i = 0, d = 0;
  PIDController leftPIDController = new PIDController(p, i, d);
  PIDController rightPIDController = new PIDController(p, i, d);

  Pose2d pose = new Pose2d();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    reset();
  }

  @Override
  public void periodic() {
    updateOdometryAndPose();

    SmartDashboard.putString("Speeds", getSpeeds().toString());
    SmartDashboard.putData("Left PID", leftPIDController);
    SmartDashboard.putData("Right PID", rightPIDController);
    SmartDashboard.putNumber("Left Motor Voltage", m_leftMotor.get());
    SmartDashboard.putNumber("Right Motor Voltage", m_rightMotor.get());
    SmartDashboard.putNumber("Left Encoder Distance", getLeftDistanceMeters());
    SmartDashboard.putNumber("Right Encoder Distance", getRightDistanceMeters());
    SmartDashboard.putNumber("Gyro X angle", m_gyro.getAngleX());
    SmartDashboard.putNumber("Gyro Y angle", m_gyro.getAngleY());
    SmartDashboard.putNumber("Gyro Z angle", m_gyro.getAngleZ());
    SmartDashboard.putNumber("Gyro Rotation", m_gyro.getRotation2d().getDegrees());
  }

  private void updateOdometryAndPose() {
    pose = odometry.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void reset() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_gyro.reset();
    updateOdometryAndPose();
  }

  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_leftEncoder.getRate(),
        m_rightEncoder.getRate()
    );
  }
  
  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public void setOutputVolts(double left, double right) {
    m_rightMotor.set(right / 12);
    m_leftMotor.set(left / 12);
  }
}
