package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class Limelight {
    private Limelight3A limelight;
    private IMU imu;



    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        //april tag pipeline changes in the limelight setup
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limelight.start();
    }

    public int getTagID() {
        //limelight.pipelineSwitch(1);
        /*LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (!fiducials.isEmpty()) {
            LLResultTypes.FiducialResult fiducial = fiducials.get(0);
            return fiducial.getFiducialId();
        } else {
            return 0;
        }*/
        return 21;
    }


    public Pose getPosition(){
        LLResult result = limelight.getLatestResult();
        limelight.pipelineSwitch(0);
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double yaw = botpose.getOrientation().getYaw();
                //telemetry.addData("limelight", "(" + x + ", " + y + ")");
               // telemetry.addData("botpose", botpose.toString());
                //final Pose limelightPose = new Pose(x, y, yaw, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
               double X = 72+(y*39.37);
               double Y = 72-(x*39.37);
               double YAW = -yaw;
               //telemetry.addData(" Pedro", "(" + X + ", " + Y + ")");
            return new Pose(X,Y,Math.toRadians(YAW));
        }else{
            return null;
        }

        //telemetry.update();
    }



  private double getDistanceFromTag(double ta) {
        //distance is the hypotenuse
        double scale = 128.9873; // = c value in equation of curve c/x
        double distance = (scale/ta) ;
        return distance;
    }


}


