package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Robot.Systems.Intake;
public class IntakeTransferSequenceCommand extends SequentialCommandGroup {
    public IntakeTransferSequenceCommand(Intake intake) {
        super(
                new IntakeClearanceTransferCommand(intake),
                new WaitCommand(300),
                new IntakeBringInCommand(intake)
        );
    }
}
