package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//adb connect 192.168.43.1

//@TeleOp(name = "Driver Control", group = "Linear Opmode")
public class DriverControl extends LinearOpMode {
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor towerYaxis1;
    private DcMotor towerYaxis2;
    private Servo claw;
    //private Servo claw2;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Running");
        telemetry.update();
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        towerYaxis1 = hardwareMap.get(DcMotor.class, "towerYaxis1");
        towerYaxis2 = hardwareMap.get(DcMotor.class, "towerYaxis2");
        claw = hardwareMap.get(Servo.class, "claw");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        runtime.reset();
        towerYaxis1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        towerYaxis2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        towerYaxis1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        towerYaxis2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y/2;
            double strafe = gamepad1.left_stick_x/2;
            double turn = gamepad1.right_stick_x/2;

            double frontLeftPower = -Range.clip(drive + strafe + turn, -1.0, 1.0);
            double frontRightPower = -Range.clip(drive - strafe - turn, -1.0, 1.0);
            double backLeftPower = -Range.clip(drive - strafe + turn, -1.0, 1.0);
            double backRightPower = Range.clip(drive + strafe - turn, -1.0, 1.0);

            front_left.setPower(frontLeftPower);
            front_right.setPower(frontRightPower);
            back_left.setPower(backLeftPower);
            back_right.setPower(-backRightPower);
            //towerYaxis1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //towerYaxis2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            towerYaxis1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            towerYaxis2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            towerYaxis1.setTargetPosition(0);
//            towerYaxis2.setTargetPosition(0);





            if(gamepad1.dpad_up){
                towerYaxis1.setPower(-1); //FIX direction
                towerYaxis2.setPower(1);
            }
            else// if(!gamepad1.dpad_up)
            {
                towerYaxis1.setPower(0);
                towerYaxis2.setPower(0);
            }


//            if(gamepad1.dpad_up){
//                towerYaxis1.setTargetPosition(towerYaxis1.getCurrentPosition() - 15); //FIX direction
//                towerYaxis2.setTargetPosition(towerYaxis2.getCurrentPosition() + 50);
//                towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                towerYaxis1.setPower(0.95);
//                towerYaxis2.setPower(0.95);
//            }
//
//
//            if(gamepad1.dpad_down){
//                towerYaxis1.setTargetPosition(towerYaxis1.getCurrentPosition() + 15); //FIX direction
//                towerYaxis2.setTargetPosition(towerYaxis2.getCurrentPosition() - 50);
//                towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                towerYaxis1.setPower(0.95);
//                towerYaxis2.setPower(0.95);
//            }


            if(gamepad1.dpad_down){
                towerYaxis1.setPower(1); //FIX direction
                towerYaxis2.setPower(-1);
//                towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else //if(!gamepad1.dpad_down)
            {
                towerYaxis1.setPower(0);
                towerYaxis2.setPower(0);
            }

//            if(gamepad1.dpad_down){
//                towerYaxis1.setTargetPosition(towerYaxis1.getCurrentPosition() + 200); //FIX direction
//                towerYaxis2.setTargetPosition(towerYaxis2.getCurrentPosition() - 200);
//            }

            while(gamepad1.right_bumper){ //closes
                claw.setPosition(claw.getPosition() - 0.005);
                //claw2.setPosition(claw2.getPosition() + 0.005);
            }

            while(gamepad1.left_bumper) {
                claw.setPosition(claw.getPosition() + 0.005);
                //claw2.setPosition(claw2.getPosition() - 0.005);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Claw Position",  claw.getPosition());
            //telemetry.addData("Claw2 Position",  claw2.getPosition());
            telemetry.addData("TowerYaxis1 Position", towerYaxis1.getCurrentPosition());
            telemetry.addData("TowerYaxis2 Position", towerYaxis2.getCurrentPosition());
            telemetry.update();
        }

    }

}

// Need code for moving arm side to side
// Need code for close and opening claw (left and right button)