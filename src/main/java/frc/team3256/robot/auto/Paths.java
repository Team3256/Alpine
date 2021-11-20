package frc.team3256.robot.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.team3256.robot.Constants;
import frc.team3256.robot.auto.paths.JSONReader;

import java.util.ArrayList;
import java.util.List;

import static frc.team3256.robot.Constants.AutoConstants.*;

public class Paths {


    public static Trajectory getTrajectory1() {
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
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)),
                        config);

        var thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
                new SwerveControllerCommand(
                        trajectory1,
                        m_robotDrive::getPose, // Functional interface to feed supplier
                        Constants.SwerveConstants.kDriveKinematics,

                        // Position controllers
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                        thetaController,
                        m_robotDrive::setModuleStates,
                        m_robotDrive);

        // Run path following command, then stop at the end.
        return trajectory1;
    }

}
