package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deposit extends SubsystemBase {

    private final Servo lof;
    private final Servo rof;
    private final Servo right_elbow;
    private final Servo o_claw;

    // Servo position constants

    public Deposit(final HardwareMap hardwareMap) {
        lof = hardwareMap.get(Servo.class, "left_outtake_flip");
        rof = hardwareMap.get(Servo.class, "right_outtake_flip");
        right_elbow = hardwareMap.get(Servo.class, "right_elbow");
        o_claw = hardwareMap.get(Servo.class, "outtake_claw");
    }

    public void setServoPosition (int servo, double position){
        switch (servo){
            case 1:
                lof.setPosition(position);
                break;
            case 2:
                rof.setPosition(position);
                break;
            case 3:
                right_elbow.setPosition(position);
                break;
            case 4:
                o_claw.setPosition(position);
                break;
        }
    }

}
