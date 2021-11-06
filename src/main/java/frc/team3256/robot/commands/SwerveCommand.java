package frc.team3256.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3256.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class SwerveCommand extends CommandBase {
    public final SwerveDrive swerveDrive;

    public final DoubleSupplier velocityX;
    public final DoubleSupplier velocityY;
    public final DoubleSupplier angularSpeed;

    public SwerveCommand(DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier angularSpeed, SwerveDrive swerveDrive){
        this.swerveDrive = swerveDrive;
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.angularSpeed = angularSpeed;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.drive(velocityX.getAsDouble(), velocityX.getAsDouble(), angularSpeed.getAsDouble());
    }

    // never stop driving lmao
    @Override
    public boolean isFinished() {
        return false;
    }

    // If the command ends, stop driving
    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(0, 0, 0);
    }
}
