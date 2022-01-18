package frc.team3256.robot.commands;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3256.robot.helper.SwerveDriveController;
import frc.team3256.robot.subsystems.SwerveDrive;
import static frc.team3256.robot.Constants.AutoConstants.*;

public class TrajectoryFollowCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private final SwerveDriveController controller;
    private final Rotation2d desiredRotationFinalRotation;
    private final SwerveDrive driveSubsystem;
    private final double trajectoryDuration;

    public TrajectoryFollowCommand(
            Trajectory trajectory,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Rotation2d desiredRotation,
            SwerveDrive driveSubsystem) {

        this.trajectory = trajectory;
        this.trajectoryDuration = trajectory.getTotalTimeSeconds();
        this.controller = new SwerveDriveController(
                xController,
                yController,
                thetaController
        );
        this.desiredRotationFinalRotation = desiredRotation;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double now = timer.get();
        Trajectory.State desired = trajectory.sample(now);
        Pose2d currentPose = driveSubsystem.getPose();
        Pose2d desiredPose = desired.poseMeters;
        double desiredLinearVelocity = desired.velocityMetersPerSecond;
        double thetaFF = this.desiredRotationFinalRotation.getRadians() / trajectoryDuration * 0.02 * THETA_FF; // 20ms loop time
        double thetaSetpoint = this.desiredRotationFinalRotation.getRadians() * (now/trajectoryDuration);
        Rotation2d desiredRotation = new Rotation2d(thetaSetpoint + thetaFF);

        SmartDashboard.putNumber("Desired Rotation", Units.radiansToDegrees(thetaSetpoint));
        SmartDashboard.putNumber("Current Rotation", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Desired Position", Units.metersToInches(desiredPose.getX()));

        driveSubsystem.drive(controller.calculate(currentPose, desiredPose, desiredLinearVelocity, desiredRotation));
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= trajectoryDuration;
    }
}
