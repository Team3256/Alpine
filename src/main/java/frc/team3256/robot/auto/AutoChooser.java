package frc.team3256.robot.auto;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3256.robot.subsystems.SwerveDrive;

public class AutoChooser {
    private static SendableChooser<Command> autoChooser;

    public static SendableChooser<Command> getDefaultChooser(SwerveDrive drive) {
        autoChooser = new SendableChooser<>();
//        Command trajectory1 = Paths.getTrajectory1(drive).andThen(() -> drive.drive(new ChassisSpeeds(0, 0, 0)));
//        autoChooser.addOption("Left", trajectory1);
        return autoChooser;
    }

    public static Command getCommand() {
        return autoChooser.getSelected();
    }
}
