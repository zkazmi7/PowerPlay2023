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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//TILE DIMENSION 24 x 24 INCH
//adb connect 192.168.43.1
@Config
@Autonomous(name = "AutonomousScanParkRight", group = "Autonomous")
public class AutoScanAndParkRight extends LinearOpMode {
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
    //public Pose2d lineUpWithOpening;
    //public DigitalChannel intakeLight; //ADD BACK
    //public Pose2d startingPose;
    //public Vector2d shippingHubPose;
    private DcMotor towerYaxis1;
    private DcMotor towerYaxis2;
    private Servo claw;
    private ColorSensor colorSensor;
    //private int sleeve;

    public void openClaw() {
        claw.setPosition(0.6);
        sleep(500);
    }

    public void closeClaw() {
        claw.setPosition(0.0);
        sleep(500);
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
                .back(4)
                .build());
    }

    public void moveForward() {
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(4) //FIX
                .build());
    }


    public void resetSlide() {
        towerYaxis1.setTargetPosition(11); //FIND OUT RESET
        towerYaxis2.setTargetPosition(-3);
    }

    public void goToConesFirst(){
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(48)
                .build());
    }

    public void goToCones(){
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(60)
                .build());
    }

    public void returnFromCones(){
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(60)
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
                .turn(-Math.PI/2)
                .strafeLeft(36)
                .build());
        placeInHighLevel();
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.PI/2)
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
                .forward(2)
                .build());
    }

    public int scanSleeve() {
        int spot = 0;
        if(colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
            return 2;
        }
        else if(colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
            return 3;
        }
        else {
            return 1;
        }
    }

    public void park(int s) {
        if ( s == 1)
        {
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                 .forward(24)
                 .strafeLeft(24)
                 .build());
        }
        else if (s == 3)
        {
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(24)
                    .strafeRight(24)
                    .build());
        }
    }

    public void realPark(int s) {
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(12)
                .strafeRight(24)
                .build());
        if(s == 1){
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeLeft(24)
                    .build());
        } else if(s == 3){
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(24)
                    .build());
        }
    }

    public void runOpMode() {
        telemetry.addData("Status", "Running");
        telemetry.update();
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
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
            //start on right side
            int sleeve = scanSleeve();
            park(sleeve);

            break;
        }

    }
}