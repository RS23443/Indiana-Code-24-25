package org.firstinspires.ftc.teamcode.Robot.Sensing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimelightBaseSystem{
    private Limelight3A limelight;
    public LLResult result;
    public double angle;
    public LimelightBaseSystem(final HardwareMap hardwareMap, final String limelightName){
        limelight = hardwareMap.get(Limelight3A.class, limelightName);
        limelight.pipelineSwitch(0);
    }

    public void startLimelight(){
        limelight.start();
    }
    public void FetchResults(){
        result = latestResult();
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    if(cr.getTargetYDegrees() == 0){
                        angle = 90;
                    } else {
                        angle = Math.atan(cr.getTargetXDegrees() / cr.getTargetYDegrees());
                        //changing the radians into degrees
                        angle = angle * 180 / Math.PI;
                    }
                }
            }
        }
    }

    public double angleOfBlock(){
        return angle;
    }

    public LLResult latestResult(){
        return limelight.getLatestResult();
    }

}
