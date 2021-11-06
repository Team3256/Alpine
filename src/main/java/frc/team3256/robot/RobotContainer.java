package frc.team3256.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3256.robot.commands.ExampleCommand;
import frc.team3256.robot.subsystems.SwerveDrive;
import frc.team3256.robot.commands.SwerveCommand;
import frc.team3256.robot.helper.JoystickAnalogButton;
import edu.wpi.first.wpilibj2.command.button.Button;

import static edu.wpi.first.wpilibj.GenericHID.Hand.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveDrive swerveDrive = new SwerveDrive();

    XboxController xboxController = new XboxController(0);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerveDrive.setDefaultCommand(new SwerveCommand( ()->xboxController.getX(kLeft), ()->xboxController.getY(kLeft), ()->xboxController.getX(kRight), swerveDrive));

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
        Button rightTrigger = new JoystickAnalogButton(xboxController, XboxController.Axis.kRightTrigger.value);
        Button leftTrigger = new JoystickAnalogButton(xboxController, XboxController.Axis.kLeftTrigger.value);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
//    public Command getAutonomousCommand() {
//        // An ExampleCommand will run in autonomous
////        return m_autoCommand;
//    }
}