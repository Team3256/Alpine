package frc.team3256.robot;

public final class Constants {
    public static class AutoConstants {
        public static double MIN_SPACE_BETWEEN_POINTS = 0.5;
        public static final double[] FRONT_LEFT = {0.5, 0.5};
        public static final double[] FRONT_RIGHT = {0.5,-0.5};
        public static final double[] BACK_LEFT = {-0.5,0.5};
        public static final double[] BACK_RIGHT = {-0.5,-0.5};
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
