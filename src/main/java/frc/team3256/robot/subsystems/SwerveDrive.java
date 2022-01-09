package frc.team3256.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.fasterxml.jackson.annotation.JsonEnumDefaultValue;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team3256.robot.Constants.SwerveConstants.*;

public class SwerveDrive extends SubsystemBase {
    public static final double MAX_VOLTAGE = 12.0;




    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACK_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACK_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACK_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACK_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
    private final Field2d field = new Field2d();
    // FIXME Remove if you are not using a Pigeon
//    private final PigeonIMU pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private Pose2d pose = new Pose2d(0, 0, new Rotation2d(0));
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), pose);

    public SwerveDrive() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // FIXME Setup motor configuration
        frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk3SwerveModuleHelper.GearRatio.STANDARD,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET
        );
        SmartDashboard.putData("Field", field);
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        // FIXME Remove if you are not using a Pigeon
//        pigeon.setFusedHeading(0.0);
    }

    public Rotation2d getGyroscopeRotation() {
        // FIXME Remove if you are using a Pigeon
        return new Rotation2d();
        // return Rotation2d.fromDegrees(-pigeon.getFusedHeading());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public Pose2d getPose() { return odometry.getPoseMeters();}

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

        SmartDashboard.putNumber("Desired Front Left Speed", desiredStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("Desired Front Right Speed", desiredStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("Desired Back Left Speed", desiredStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("Desired Back Right Speed", desiredStates[3].speedMetersPerSecond);

        SmartDashboard.putNumber("Desired Front Left Angle", desiredStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Desired Front Right Angle", desiredStates[1].angle.getDegrees());
        SmartDashboard.putNumber("Desired Back Left Angle", desiredStates[2].angle.getDegrees());
        SmartDashboard.putNumber("Desired Back Right Angle", desiredStates[3].angle.getDegrees());

        frontLeftModule.set(desiredStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[0].angle.getRadians());
        frontRightModule.set(desiredStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[1].angle.getRadians());
        backLeftModule.set(desiredStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[2].angle.getRadians());
        backRightModule.set(desiredStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[3].angle.getRadians());
    }

    public void outputToDashboard() {
        SmartDashboard.putData("Field", field);

        SmartDashboard.putNumber("Front Left Speed", frontLeftModule.getDriveVelocity());
        SmartDashboard.putNumber("Front Right Speed", frontRightModule.getDriveVelocity());
        SmartDashboard.putNumber("Back Left Speed", backLeftModule.getDriveVelocity());
        SmartDashboard.putNumber("Back Right Speed", backRightModule.getDriveVelocity());

        SmartDashboard.putNumber("Front Left Angle", frontLeftModule.getSteerAngle());
        SmartDashboard.putNumber("Front Right Angle", frontRightModule.getSteerAngle());
        SmartDashboard.putNumber("Back Left Angle", backLeftModule.getSteerAngle());
        SmartDashboard.putNumber("Back Right Angle", backRightModule.getSteerAngle());
    }

    @Override
    public void periodic() {
        Rotation2d gyroAngle = getGyroscopeRotation();
        // Update the pose
        SwerveModuleState frontLeftState = new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle()));
        SwerveModuleState frontRightState = new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle()));
        SwerveModuleState backLeftState = new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle()));
        SwerveModuleState backRightState = new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()));

        pose = odometry.update(gyroAngle, frontLeftState, frontRightState,
                backLeftState, backRightState);
        field.setRobotPose(pose);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);

        outputToDashboard();
    }
}