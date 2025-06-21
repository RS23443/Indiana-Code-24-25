package pedroPathing.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import pedroPathing.Robot.Constants;
import pedroPathing.Robot.Systems.Intake;


public class Intake45Degrees extends SequentialCommandGroup {
    public final Intake intake;
    public Intake45Degrees(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

        addCommands(
                new InstantCommand(() -> {
                    intake.setServoPosition(1, Constants.intakeActive[0]);
                    intake.setServoPosition(2, Constants.intakeActive[1]);
                }, intake),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    intake.setServoPosition(3, Constants.intakeHorizontalSpin[0]);
                    intake.setServoPosition(4, Constants.intakeHorizontalSpin[1]);
                }, intake),
                new WaitCommand(200),
                new InstantCommand(() -> intake.setServoPosition(5, Constants.intakeActive[4]), intake)
        );
    }
}