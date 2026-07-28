package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous
public class Test2 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Limelight3A limelight;
    private IMU imu;
    private int pathState;
    private final Pose startPose = new Pose(34, 135, Math.toRadians(0)); // Start Pose currently left corner subject to change
    private final Pose scorePose = new Pose(53, 87 ,Math.toRadians(-45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //test this seems ok
    private final Pose setup1Pose = new Pose(50,84,Math.toRadians(180)); // setup for the following pose
    private final Pose pickup1Pose = new Pose(22, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose setup2Pose = new Pose(50,60,Math.toRadians(180)); // setup for the following pose
    private final Pose pickup2Pose = new Pose(22, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose setup3Pose = new Pose(50,36,Math.toRadians(180)); // setup for the following pose
    private final Pose pickup3Pose = new Pose(22, 36, Math.toRadians(180));// Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose endPose = new Pose(56, 18, Math.toRadians(-68));// Lowest (Third Set) of Artifacts from the Spike Mark.
    private Path scorePreload;
    private Pose correction;
    private PathChain grabPickup1, goSetup1, scorePickup1, grabPickup2, goSetup2, scorePickup2,grabPickup3, goSetup3, scorePickup3, adjustment;
    long millis;
    int line = 0;
    double outtakepower = -0.1;
    long wait = 400;
    double out = 1840;



    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        goSetup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, setup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), setup1Pose.getHeading())
                .build();
        //path to the middle point
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(setup1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(setup1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        goSetup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, setup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), setup2Pose.getHeading())
                .build();
        //path to the middle point
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(setup2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(setup2Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        goSetup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, setup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), setup3Pose.getHeading())
                .build();
        //path to the middle point
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(setup3Pose, pickup3Pose))
                .setLinearHeadingInterpolation(setup3Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, endPose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(11);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goSetup1, true);
                    setPathState(2);
                }
                break;


            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup1, true);

                    setPathState(3);
                }
                break;
            case 3:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup1,true);
                    sleep(wait);
                    setPathState(11);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goSetup2, true);
                    setPathState(5);
                }
                break;


            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup2, true);


                    setPathState(6);
                }
                break;
            case 6:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup2,true);
                    sleep(wait);
                    setPathState(11);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goSetup3, true);
                    setPathState(8);
                }
                break;


            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup3, true);
                    setPathState(9);
                }
                break;
            case 9:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scorePickup3,true);
                    sleep(wait);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */





                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */

                    setPathState(-1);
                }
                break;

            case 11:
                if(!follower.isBusy()) {
                    correction = getRobotPoseFromCamera();
                    adjustment = follower.pathBuilder()
                            .addPath(new BezierLine(correction, scorePose))
                            .setLinearHeadingInterpolation(correction.getHeading(), scorePose.getHeading())
                            .build();
                    follower.followPath(adjustment,true);
                    if (line == 2) {
                        line += 1;
                        setPathState(7);
                    }
                    if (line == 1) {
                        line += 1;
                        setPathState(4);
                    }
                    if (line == 0) {
                        line += 1;
                        setPathState(1);
                    }


                }

break;

        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);



        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        buildPaths();



    }
    private Pose getRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose()                       for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            double x = botpose.getPosition().x;
            double y= botpose.getPosition().y;
            double yaw = botpose.getOrientation().getYaw();
            double X = 72+(y*39.37);
            double Y = 72-(x*39.37);

            telemetry.addLine("valid");
            telemetry.addData(" Pedro", "(" + X + ", " + Y + ")");
            return new Pose(X, Y, yaw, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }else{
            telemetry.addLine("Not valid");
            return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        }


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}