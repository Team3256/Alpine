package frc.team3256.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.team3256.robot.Constants;

public class SwerveModule {

    private final TalonFX m_driveMotor;
    private final TalonFX m_turningMotor;
    private final Encoder m_driveEncoder;
    private final Encoder m_turningEncoder;

//    private final Encoder m_driveEncoder;
//    private final Encoder m_turningEncoder;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController =
            new ProfiledPIDController(
                    1,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(
                            Constants.SwerveModuleConstants.kModuleMaxAngularVelocity, Constants.SwerveModuleConstants.kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorChannel PWM output for the drive motor.
     * @param turningMotorChannel PWM output for the turning motor.
     */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
        m_driveMotor = new TalonFX(driveMotorChannel);
        m_turningMotor = new TalonFX(turningMotorChannel);

        m_driveEncoder = new Encoder(1,1);
        m_turningEncoder = new Encoder(1,1);
        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_driveEncoder.setDistancePerPulse(2 * Math.PI * Constants.SwerveModuleConstants.kWheelRadius / Constants.SwerveModuleConstants.kEncoderResolution);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        m_turningEncoder.setDistancePerPulse(2 * Math.PI / Constants.SwerveModuleConstants.kEncoderResolution);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.get()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
                SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
                m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
                m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

        final double turnFeedforward =
                m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

//        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
//        m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    }
}
