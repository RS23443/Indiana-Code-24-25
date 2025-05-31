package org.firstinspires.ftc.teamcode.Robot.Systems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.PIDController;

public class Intake extends SubsystemBase {
    public double kP = 0.01, kI = 0.00, kD = 0.000, alpha = 0;
    public  PIDController controller = new PIDController(kP, kI, kD, alpha);
    private final Servo lif;
    private final Servo rif;
    private final Servo rightdiffy;
    private final Servo leftdiffy;
    private final DcMotorEx horizontalExtension;
    private final Servo claw;


    public Intake(final HardwareMap hardwareMap) {
        lif = hardwareMap.get(Servo.class, "left_intake_flip");
        rif = hardwareMap.get(Servo.class, "right_intake_flip");
        leftdiffy = hardwareMap.get(Servo.class, "left_differential");
        rightdiffy = hardwareMap.get(Servo.class, "right_differential");
        claw = hardwareMap.get(Servo.class, "intake_claw");
        horizontalExtension = hardwareMap.get(DcMotorEx.class, "horizontal_extension");
        horizontalExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //horizontalExtension.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setServoPosition (int servo, double position){
        switch (servo){
            case 1:
                lif.setPosition(position);
                break;
            case 2:
                rif.setPosition(position);
                break;
            case 3:
                leftdiffy.setPosition(position);
                break;
            case 4:
                rightdiffy.setPosition(position);
                break;
            case 6:
                claw.setPosition(position);
                break;
        }
    }

    public void setMotor(int target){
        double power = controller.compute(horizontalExtension.getCurrentPosition(),target);
        horizontalExtension.setPower(power);
    }

    public void setMotorPower(double power){
        horizontalExtension.setPower(power);
    }

    public int getExtensionPosition(){
        return horizontalExtension.getCurrentPosition();
    }

    public void resetExtension(){
        horizontalExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

