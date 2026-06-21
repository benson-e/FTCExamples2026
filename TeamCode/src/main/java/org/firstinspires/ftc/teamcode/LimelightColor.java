package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous
public class LimelightColor extends OpMode {
    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(1); //whatever pipeline set up for color (yellow) detection
    }

    @Override
    public void start() {
        limelight.start();
    } //not in init to save energy if delay put in init

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null & llResult.isValid()){
            telemetry.addData("Target x offset", llResult.getTx());
            telemetry.addData("Target y Offset", llResult.getTy());
            telemetry.addData("Target Area Offset", llResult.getTa());
        }
    }
}
