package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectoryCommand extends RamseteCommand {
    private Drivetrain drivetrain;

    public FollowTrajectoryCommand(Drivetrain drivetrain, Trajectory trajectory) {
        super(
            trajectory,
            drivetrain::getPose,
            new RamseteController(2, .7),
            drivetrain.getFeedforward(),
            drivetrain.getKinematics(),
            drivetrain::getSpeeds,
            drivetrain.getLeftPIDController(),
            drivetrain.getRightPIDController(),
            drivetrain::setOutputVolts,
            drivetrain
        );
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.reset();
        drivetrain.setDriveSafety(false); // https://www.chiefdelphi.com/t/bug-or-misunderstanding-in-trajectory-tutorial-and-ramsete-example/373989/5
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.setDriveSafety(true);
        drivetrain.setOutputVolts(0, 0);
    }
}
