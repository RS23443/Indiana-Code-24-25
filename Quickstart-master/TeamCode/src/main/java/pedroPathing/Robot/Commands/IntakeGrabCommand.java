package pedroPathing.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import pedroPathing.Robot.Constants;
import pedroPathing.Robot.Systems.Intake;

public class IntakeGrabCommand extends SequentialCommandGroup {
    public final Intake intake;

    public IntakeGrabCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

        addCommands(
                new InstantCommand(()-> {intake.setServoPosition(1,Constants.intakeActive[0]); intake.setServoPosition(2,Constants.intakeActive[1]);}, intake),
                new InstantCommand(() -> intake.setServoPosition(6, Constants.intakeSampleReset[4]), intake),
                new WaitCommand(150),
                new InstantCommand(() -> {
                    intake.setServoPosition(1,0.6);
                    intake.setServoPosition(2,0.4);
                }, intake),
                new WaitCommand(200),

                new InstantCommand(() -> {
                    intake.setServoPosition(3,0.15);
                    intake.setServoPosition(4,0.85);
                }, intake)

        );
    }
}