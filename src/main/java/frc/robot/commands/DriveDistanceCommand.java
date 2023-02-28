package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistanceCommand extends CommandBase {
    private Drivetrain drivetrain;
    private double distanceMeters;
    private double startingDistance;
    private double startingRotation;

    private PIDController speedController;
    private PIDController rotationController;
    private Gyro gyro;

    public DriveDistanceCommand(double distanceMeters, Drivetrain drivetrain, Gyro gyro) {
        super();
        this.distanceMeters = distanceMeters;
        this.drivetrain = drivetrain;
        this.gyro = gyro;

        speedController = new PIDController(10, 0, 0);
        rotationController = new PIDController(.03, 0, 0);
    }

    private double getAverageDistance() {
        return (this.drivetrain.getLeftDistanceMeters() + this.drivetrain.getRightDistanceMeters()) / 2;
    }

    @Override
    public void initialize() {
        super.initialize();

        startingDistance = getAverageDistance();
        startingRotation = gyro.getAngle();
    }

    @Override
    public void execute() {
        super.execute();

        var distanceToGo = (distanceMeters - startingDistance) - (getAverageDistance() - startingDistance);
        var speed = speedController.calculate(distanceToGo);

        var rotationError = startingRotation - gyro.getAngle();
        var rotation = rotationController.calculate(rotationError);

        drivetrain.arcadeDrive(-speed, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        drivetrain.setOutputVolts(0, 0);
    }
}
