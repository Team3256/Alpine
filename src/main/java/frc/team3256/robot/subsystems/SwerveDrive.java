package frc.team3256.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3256.robot.Constants.AutoConstants;

public class SwerveDrive extends SubsystemBase {
    private final Translation2d frontLeft = new Translation2d(AutoConstants.FRONT_LEFT[0], AutoConstants.FRONT_LEFT[1]);
    private final Translation2d frontRight = new Translation2d(AutoConstants.FRONT_RIGHT[0], AutoConstants.FRONT_RIGHT[1]);
    private final Translation2d backLeft = new Translation2d(AutoConstants.BACK_LEFT[0], AutoConstants.BACK_LEFT[1]);
    private final Translation2d backRight = new Translation2d(AutoConstants.BACK_RIGHT[0], AutoConstants.BACK_RIGHT[1]);

    //TODO: change CAN channels for modules
    private final SwerveModule m_frontLeftModule = new SwerveModule(0,0);
    private final SwerveModule m_frontRightModule = new SwerveModule(0,0);
    private final SwerveModule m_backLeftModule = new SwerveModule(0,0);
    private final SwerveModule m_back_RightModule = new SwerveModule(0,0);

    private Pose2d m_pose = new Pose2d(5.0, 13.5, new Rotation2d());

    private AnalogGyro m_gyro = new AnalogGyro(0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);

    private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroHeading(), m_pose);

    private Rotation2d gyroAngle;

    private Rotation2d getGyroHeading() {
        return gyroAngle;
    }

    /**
     * Creates a new ExampleSubsystem.
     */
    public SwerveDrive() {
        m_gyro.reset();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());

        // Update the pose
        m_pose = m_odometry.update(gyroAngle, m_frontLeftModule.getState(), m_frontRightModule.getState(),
                m_backLeftModule.getState(), m_back_RightModule.getState());
    }

    /**
    * Drives a swerve drive drivebase using {@link SwerveDriveKinematics}
    * @param vx The velocity in the x direction relative to the field
    * @param vy The velocity in the y direction relative to the field
    * @param omega The angular velocity in rad/s of the robot
     */
    public void drive(double vx, double vy, double omega) {
        // takes in vx, vy, and omega (angular velocity)
        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);

        //converts the chassis speed to an array of module states that give the speed and angle
        SwerveModuleState[] mStates = m_kinematics.toSwerveModuleStates(speeds);
    }


}
