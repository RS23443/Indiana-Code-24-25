package pedroPathing.Robot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import pedroPathing.Robot.Constants;
import pedroPathing.Robot.Systems.Intake;

public class IntakeGrabAuto extends SequentialCommandGroup {
    public final Intake intake;

    public IntakeGrabAuto(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

        addCommands(
                new InstantCommand(() -> {
                    intake.setServoPosition(1,0.88);
                    intake.setServoPosition(2,0.12);
                }, intake),
                new WaitCommand(150),
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
                    intake.setServoPosition(6,0.45);
                }, intake)

        );
    }
}