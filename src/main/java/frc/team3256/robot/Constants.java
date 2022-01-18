package frc.team3256.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
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
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(168.8379); //357

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(233.1738); //179

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 11;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 13;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(349.8926);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 14;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 16;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(52.8223); //179

        public static final double MAX_METERS_PER_SECOND = 10;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 1380.0 / 60.0 *
         SdsModuleConfigurations.MK4_L2.getDriveReduction() *
         SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
            // 6380.0 / 60.0 *
                // SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_TRACK_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
    }
    public static class AutoConstants {
        public static double MIN_SPACE_BETWEEN_POINTS = 0.5;

        public static double MAX_SPEED_CONTROLLER_METERS_PER_SECOND = 2;
        public static double MAX_ACCELERATION_CONTROLLER_METERS_PER_SECOND_SQUARED = 2;
        public static TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(2.5 * Math.PI, 1.5 * Math.PI);

        public static double P_X_CONTROLLER = 2.2;
        public static double I_X_CONTROLLER = 0.025;
        public static double D_X_CONTROLLER = 0;

        public static double P_Y_CONTROLLER = 2.2;
        public static double I_Y_CONTROLLER = 0.025;
        public static double D_Y_CONTROLLER = 0;

        public static double TRANSLATION_FF = 0.3;

        public static double P_THETA_CONTROLLER = 2.2;
        public static double I_THETA_CONTROLLER = 0;
        public static double D_THETA_CONTROLLER = 0;
        public static double THETA_FF = 7.5;
    }

    public static class CANConstants {
        public static final int[] SparkMaxIDs = new int[]{};
        public static final int[] TalonFXIDs = new int[]{};
        public static final int pigeonID = SwerveConstants.DRIVETRAIN_PIGEON_ID;
    }

//    public static class IDConstants {
//        public static final int pigeonID = 0;
//    }
//
//    public static class LoggingConstants { }
//
//    public static class SwerveModuleConstants {
//        public static final double kWheelRadius = 0.0508;
//        public static final int kEncoderResolution = 4096;
//
//        public static final double kModuleMaxAngularVelocity = 0;
//        public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
//    }
}
