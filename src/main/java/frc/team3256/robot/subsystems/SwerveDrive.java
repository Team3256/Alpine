package frc.team3256.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.team3256.robot.constants.AutoConstants;

public class SwerveDrive {
    Translation2d frontLeft = new Translation2d(AutoConstants.kFrontLeft[0], AutoConstants.kFrontLeft[1]);
    Translation2d frontRight = new Translation2d(AutoConstants.kFrontRight[0], AutoConstants.kFrontRight[1]);
    Translation2d backLeft = new Translation2d(AutoConstants.kBackLeft[0], AutoConstants.kBackLeft[1]);
    Translation2d backRight = new Translation2d(AutoConstants.kFrontRight[0], AutoConstants.kFrontRight[1]);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);

    // takes in vx, vy, and omega (angular velocity)
    ChassisSpeeds speeds = new ChassisSpeeds(1.0,1.0,1.0);

    //converts the chassis speed to an array of module states that give the speed and angle
    SwerveModuleState[] mStates = m_kinematics.toSwerveModuleStates(speeds);



}
