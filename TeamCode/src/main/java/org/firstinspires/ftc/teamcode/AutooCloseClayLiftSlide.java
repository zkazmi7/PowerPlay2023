package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Config
@Autonomous
public class AutooCloseClayLiftSlide extends LinearOpMode
{
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DistanceSensor dSensorRight = null;
    HardwareMap hwMap = null;
    BNO055IMU imu;
    Orientation angles;
    public SampleMecanumDrive drive;
    private DcMotor towerYaxis1;
    private DcMotor towerYaxis2;
    private Servo claw;
    private ColorSensor colorSensor;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;



    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.0381;

    //tags
    int left = 1;
    int middle = 2;
    int right = 3;
    //from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    public void openClaw() {
        claw.setPosition(0.77166666666666);
        sleep(500);
    }

    public void closeClaw() {
        claw.setPosition(0.6599999999999999999);
        sleep(500);
    }

    //tower one HIGH triangle -4224 medium circle -3194 low cross -1789 auto -643
    //tower two high 4147  medium 3140  low 1765  auto 656
    public void slideLil() {
        towerYaxis1.setTargetPosition(-643); //FIND OUT HIGH
        towerYaxis2.setTargetPosition(656); //FIND OUT HIGH
        towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void slideLow() {
        towerYaxis1.setTargetPosition(-1625); //FIND OUT HIGH
        towerYaxis2.setTargetPosition(1595); //FIND OUT HIGH
        towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void slideMid() {
        towerYaxis1.setTargetPosition(-2777); //FIND OUT HIGH
        towerYaxis2.setTargetPosition(2729); //FIND OUT HIGH
        towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void slideHigh() {
        towerYaxis1.setTargetPosition(-3811); //FIND OUT HIGH
        towerYaxis2.setTargetPosition(3742); //FIND OUT HIGH
        towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void placeInHighLevel(){
        slideHigh();
        moveForward(); // may need to move more
        resetSlide();
        openClaw();
        slideHigh();
        moveBack();
        resetSlide();
    }

//    public void placeInMiddleLevel(){
//        towerYaxis.setTargetPosition(4950); //FIND OUT MIDDLE
    //        towerYaxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        openClaw();
//        moveLeft();
//        resetSlide();
//    }

//    public void placeInLowLevel(){
//        towerYaxis.setTargetPosition(3580); //FIND OUT LOW
//        towerYaxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        openClaw();
//        moveLeft();
//        resetSlide();
//    }

    public void moveBack() {
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(8)
                .build());
    }

    public void moveForward() {
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(8) //FIX
                .build());
    }


    public void resetSlide() {
        towerYaxis1.setTargetPosition(11); //FIND OUT RESET
        towerYaxis2.setTargetPosition(-3);
    }

    public void goToConesFirst(){
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(48*2)
                .build());
    }

    public void goToCones(){
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(120)
                .build());
    }

    public void returnFromCones(){
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(120)
                .build());
    }

    public void pickUpCone(int c) {
        openClaw();
        //sleep(500);
        if(c == 1){
            towerYaxis1.setTargetPosition(1000);
            towerYaxis2.setTargetPosition(600);
        }else if(c == 2){
            towerYaxis1.setTargetPosition(900);
            towerYaxis2.setTargetPosition(600);
        }else if(c == 3) {
            towerYaxis1.setTargetPosition(800);
            towerYaxis2.setTargetPosition(600);
        }else if(c == 4) {
            towerYaxis1.setTargetPosition(700);
            towerYaxis2.setTargetPosition(600);
        }else if(c == 5) {
            towerYaxis1.setTargetPosition(600);
            towerYaxis2.setTargetPosition(600);
        }
        closeClaw();
        //sleep(500);
        towerYaxis1.setTargetPosition(1100);
        towerYaxis2.setTargetPosition(1100);
    }

    public void placeCone(int c){
        goToCones();
        pickUpCone(c);
        returnFromCones();
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.PI/2)
                .strafeRight(36*2)
                .build());
        placeInHighLevel();
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(-Math.PI/2)
                .build());
    }

    public void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }


    public void park() {
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(4)
                .build());
    }

    public void park (int t) {
            if ( t == 3)
            {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .strafeRight(50)
                        .build());
            }
            else if (t == 1)
            {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .strafeLeft(50)
                        .build());
            }
        }


    public void runOpMode() {
        float minPosition = 0.3f;
        float maxPosition = 0.8f;
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        //servo = hardwareMap.get(Servo.class, "servo" );

        telemetry.addData("Status", "Running");
        telemetry.update();
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        towerYaxis1 = hardwareMap.get(DcMotor.class, "towerYaxis1");
        towerYaxis2 = hardwareMap.get(DcMotor.class, "towerYaxis2");
        //clawYaxis = hardwareMap.get(Servo.class, "clawYaxis");
        claw = hardwareMap.get(Servo.class, "claw");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        int ses = 0;
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLD();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == left || tag.id == middle || tag.id == right) {
                        tagOfInterest = tag;
                        if(tagOfInterest.id == left)
                            ses = 1;
                        if(tagOfInterest.id == right)
                            ses = 3;
                        if(tag.id == left)
                            ses = 1;
                        if(tag.id == right)
                            ses = 3;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    if(tagOfInterest.id == left)
                        ses = 1;
                    if(tagOfInterest.id == right)
                        ses = 3;
                } else {
                    telemetry.addLine("no tag boi");
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");


            }

            telemetry.update();
            sleep(20);
        }


        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, never sighted(");
            telemetry.update();
        }
        telemetry.addData("Status", "Running");
        telemetry.update();
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        towerYaxis1 = hardwareMap.get(DcMotor.class, "towerYaxis1");
        towerYaxis2 = hardwareMap.get(DcMotor.class, "towerYaxis2");
        //clawYaxis = hardwareMap.get(Servo.class, "clawYaxis");
        claw = hardwareMap.get(Servo.class, "claw");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);

        this.stopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        towerYaxis1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        towerYaxis2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //intakeLight.setMode(DigitalChannel.Mode.OUTPUT);
        towerYaxis1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        towerYaxis2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //make sure imu is mounted flat on robot
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("hello", "hi");
        telemetry.addData("claw1 " , claw.getPosition());
        telemetry.update();



        waitForStart();
        while(opModeIsActive()){
            closeClaw();
            slideLil();

            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(32.5)
                    .build());
            park(ses);

            stopDriving();
            sleep(5000);
            break;
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}