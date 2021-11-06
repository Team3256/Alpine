package frc.team3256.robot;

public final class Constants {
    public static class AutoConstants {
        public static double MIN_SPACE_BETWEEN_POINTS = 0.5;
        public static final double[] kFrontLeft = {0.5, 0.5};
        public static final double[] kFrontRight = {0.5,-0.5};
        public static final double[] kBackLeft = {-0.5,0.5};
        public static final double[] kBackRight = {-0.5,-0.5};
    }

    public static class CANConstants {
        public static final int[] SparkMaxIDs = new int[]{};
        public static final int[] TalonFXIDs = new int[]{};
    }

    public static class IDConstants {
        public static final int pigeonID = 0;
    }

    public static class LoggingConstants { }
}
