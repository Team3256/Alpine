package frc.team3256.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public final class Constants {
    public static class SwerveConstants {
        public static final double MAX_VOLTAGE = 12.0;

        public static final double DRIVETRAIN_TRACK_METERS = 0.4445;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4445;

        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(168.8379);
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(233.1738);
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(349.8926);
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(52.8223);

        private static final double MOTOR_FREE_SPIN_RPM = 6380.0;

        // Calculated Values (Don't Change)
        public static final double MAX_VELOCITY_METERS_PER_SECOND = MOTOR_FREE_SPIN_RPM / 60.0 *
                SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_TRACK_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
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

    public static class IDConstants {
        public static final int[] TALON_FX_IDS = new int[]{5,6,8,9,11,12,14,15};
        public static final int[] SPARK_MAX_IDS = new int[]{};

        public static final int DRIVETRAIN_PIGEON_ID = 4;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 5;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 6;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 7;

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 8;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 9;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 10;

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 11;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 12;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 13;

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 14;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 15;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 16;


    }
}
