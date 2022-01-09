package frc.team3256.robot.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.team3256.robot.Constants;
import frc.team3256.robot.auto.paths.JSONReader;
import frc.team3256.robot.commands.DefaultDriveCommand;
import frc.team3256.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.List;

import static frc.team3256.robot.Constants.AutoConstants.*;

public class Paths {
    public static Command getTrajectory1(SwerveDrive robotDrive) {
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.SwerveConstants.kDriveKinematics);

        List<Translation2d> waypoints = JSONReader.ParseJSONFile("");

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory1 =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        waypoints,
                        // End 200 inches straight ahead of where we started, facing forward
                        new Pose2d(Units.inchesToMeters(200), 0, new Rotation2d(0)),
                        config);

        var thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory1,
                robotDrive::getPose, // Functional interface to feed supplier
                Constants.SwerveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                robotDrive::setModuleStates,
                robotDrive);

        return swerveControllerCommand.andThen(() -> robotDrive.drive(new ChassisSpeeds()));
    }

    public static SwerveControllerCommand getTrajectory2(SwerveDrive robotDrive) {
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.SwerveConstants.kDriveKinematics);

        List<Translation2d> waypoints = JSONReader.ParseJSONFile("");

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory2 =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        waypoints,
                        // End 200 inches straight ahead of where we started, facing forward
                        new Pose2d(Units.inchesToMeters(200), 0, new Rotation2d(0)),
                        config);

        var thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SwerveControllerCommand(
            trajectory2,
            robotDrive::getPose, // Functional interface to feed supplier
            Constants.SwerveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            robotDrive::setModuleStates,
            robotDrive);
    }

    public static SwerveControllerCommand getTrajectory3(SwerveDrive robotDrive) {
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.SwerveConstants.kDriveKinematics);

        List<Translation2d> waypoints = JSONReader.ParseJSONFile("");

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory3 =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        waypoints,
                        // End 200 inches straight ahead of where we started, facing forward
                        new Pose2d(Units.inchesToMeters(200), 0, new Rotation2d(0)),
                        config);

        var thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SwerveControllerCommand(
            trajectory3,
            robotDrive::getPose, // Functional interface to feed supplier
            Constants.SwerveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            robotDrive::setModuleStates,
            robotDrive);
    }
}
