package frc.team3256.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3256.robot.Constants.AutoConstants.*;

public class SwerveDrive extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public SwerveDrive() { }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    /**
    * Drives a swerve drive drivebase using {@link SwerveDriveKinematics}
    * @param vx The velocity in the x direction relative to the field
    * @param vy The velocity in the y direction relative to the field
    * @param omega The angular velocity in rad/s of the robot
     */
    public void drive(double vx, double vy, double omega) {
        Translation2d frontLeft = new Translation2d(kFrontLeft[0], kFrontLeft[1]);
        Translation2d frontRight = new Translation2d(kFrontRight[0], kFrontRight[1]);
        Translation2d backLeft = new Translation2d(kBackLeft[0], kBackLeft[1]);
        Translation2d backRight = new Translation2d(kBackRight[0], kBackRight[1]);

        SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);

        // takes in vx, vy, and omega (angular velocity)
        ChassisSpeeds speeds = new ChassisSpeeds(1.0,1.0,1.0);

        //converts the chassis speed to an array of module states that give the speed and angle
        SwerveModuleState[] mStates = m_kinematics.toSwerveModuleStates(speeds);
    }




}
