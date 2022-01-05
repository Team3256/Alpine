package frc.team3256.robot;

import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public final class Constants {
    public static class SwerveConstants {
        public static final double DRIVETRAIN_TRACK_METERS = 0.4445;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4445;

        public static final int DRIVETRAIN_PIGEON_ID = 4; // FIXME get a pigeon lmao

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 7;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(6.0);

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(50.0);

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 11;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 13;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(3.0);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 14;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 16;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(53.0);

        public static final double MAX_METERS_PER_SECOND = 10;
        public static SwerveDriveKinematics kDriveKinematics;
    }
    public static class AutoConstants {
        public static double MIN_SPACE_BETWEEN_POINTS = 0.5;
        public static final double[] FRONT_LEFT = {0.5, 0.5};
        public static final double[] FRONT_RIGHT = {0.5,-0.5};
        public static final double[] BACK_LEFT = {-0.5,0.5};
        public static final double[] BACK_RIGHT = {-0.5,-0.5};
        public static double P_THETA_CONTROLLER;
        public static double kMaxSpeedMetersPerSecond;
        public static double kMaxAccelerationMetersPerSecondSquared;
        public static double kPXController;
        public static TrapezoidProfile.Constraints kThetaControllerConstraints;
        public static double kPYController;
    }

    public static class CANConstants {
        public static final int[] SparkMaxIDs = new int[]{};
        public static final int[] TalonFXIDs = new int[]{};
    }

    public static class IDConstants {
        public static final int pigeonID = 0;
    }

    public static class LoggingConstants { }

    public static class SwerveModuleConstants {
        public static final double kWheelRadius = 0.0508;
        public static final int kEncoderResolution = 4096;

        public static final double kModuleMaxAngularVelocity = 0;
        public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
    }
}
