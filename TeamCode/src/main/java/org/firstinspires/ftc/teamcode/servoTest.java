package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.servo;

@TeleOp
public class servoTest extends OpMode {
    servo bench = new servo();

    @Override
    public void init(){
        bench.init(hardwareMap);

    }

    @Override
    public void loop(){
        if (gamepad1.dpad_left) {
            bench.setServoPos(0);
        }
        else {
            bench.setServoPos(1.0);
        }
        if (gamepad1.b) {
            bench.setServoRot(1.0);
        }
        else {
            bench.setServoRot(0);
        }
        if(gamepad1.a){
            bench.setServoRot(-1.0);
        }
        else {
            bench.setServoRot(0);
        }

    }





}
