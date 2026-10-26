package org.firstinspires.ftc.teamcode.mechanisms.teleOP;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

public class fieldOriented {


    @TeleOp
    public class FieldOriented extends OpMode {
        MecanumDrive drive = new MecanumDrive();
        double forward, strafe, rotate;

        double intakePower, midtakePower, sidetakePower, gateAngle, flywheelPower, turretPower;
        public AnalogInput floodgate;
        Limelight3A limelight;
        private IMU imu;


        @Override
        public void init() {
            drive.init(hardwareMap);
            //manualTurret.init(hardwareMap);

        }

        @Override
        public void loop() {
            forward = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;



            drive.driveFieldRelative(forward, strafe, rotate);

        }
    }
}
