package frc.team3256.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.team3256.robot.auto.AutoChooser;
import frc.team3256.robot.commands.DefaultDriveCommand;
import frc.team3256.robot.subsystems.SwerveDrive;
import frc.team3256.robot.Constants.SwerveConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveDrive drivetrainSubsystem = new SwerveDrive();

    private final XboxController controller = new XboxController(0);

    /**
     *
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotationx

       drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
                drivetrainSubsystem,
                () -> -modifyAxis(controller.getY(GenericHID.Hand.kLeft)) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(controller.getX(GenericHID.Hand.kLeft)) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(controller.getX(GenericHID.Hand.kRight)) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Back button zeros the gyroscope
        new Button(controller::getAButton)
                // No requirements because we don't need to interrupt anything
                .whenPressed(drivetrainSubsystem::zeroGyroscope);
    }
    public SendableChooser<Command> getCommandChooser() {
        return AutoChooser.getDefaultChooser(drivetrainSubsystem);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoChooser.getCommand();
    }
    public void resetPose() {
        drivetrainSubsystem.resetOdometry(new Pose2d());
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}