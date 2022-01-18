package frc.team3256.robot.auto;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3256.robot.commands.DefaultDriveCommand;
import frc.team3256.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class AutoChooser {
    private static SendableChooser<Command> autoChooser;

    public static SendableChooser<Command> getDefaultChooser(SwerveDrive drive) {
        autoChooser = new SendableChooser<>();

        Command doNothing = new DefaultDriveCommand(drive); // dont move
        autoChooser.setDefaultOption("Do Nothing rip :(((((", doNothing);

        Command trajectory1 = Paths.getTrajectory1(drive);
        autoChooser.addOption("SwerveControllerCommand Follower (BAd)", trajectory1);

        Command trajectory2 = Paths.getTrajectory2(drive);
        autoChooser.addOption("Custom Path Follower", trajectory2);
//
//        Command trajectory3 = Paths.getTrajectory3(drive);
//        autoChooser.addOption("Traj 3", trajectory3);

        return autoChooser;
    }

    public static Command getCommand() {
        return autoChooser.getSelected();
    }
}
