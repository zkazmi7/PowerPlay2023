package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//adb connect 192.168.43.1

@TeleOp(name = "Driver Control NL", group = "Opmode")
public class DriverControlNonLinear extends OpMode {
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor towerYaxis1;
    private DcMotor towerYaxis2;
    private Servo claw;
    private ColorSensor colorSensor;
    //private Servo claw2;
    int debugTimes = 0;
    //@Override
    public void init() {

        telemetry.addData("Status", "Running");
        telemetry.update();
        front_left = hardwareMap.get(DcMotor.class, "frontLeft");
        front_right = hardwareMap.get(DcMotor.class, "frontRight");
        back_left = hardwareMap.get(DcMotor.class, "backLeft");
        back_right = hardwareMap.get(DcMotor.class, "backRight");
        towerYaxis1 = hardwareMap.get(DcMotor.class, "towerYaxis1");
        towerYaxis2 = hardwareMap.get(DcMotor.class, "towerYaxis2");
        claw = hardwareMap.get(Servo.class, "claw");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        towerYaxis1.setDirection(DcMotor.Direction.FORWARD);
        towerYaxis2.setDirection(DcMotor.Direction.FORWARD);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);

        //waitForStart();
        runtime.reset();
        towerYaxis1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        towerYaxis2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        towerYaxis1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        towerYaxis2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
       @Override
        public void loop() {
            int sleeveNumber = 0;
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
            towerYaxis1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            towerYaxis2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            towerYaxis1.setTargetPosition(0);
//            towerYaxis2.setTargetPosition(0);


           if(colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
               telemetry.addData("color","red");
               //return 2;
           }
           else if(colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
               telemetry.addData("color","blue");
               //return 3;
           }
           else {
               telemetry.addData("color","green");
               //return 1;
           }
           //
           //

            if(gamepad1.dpad_up){
                towerYaxis1.setPower(-1); //FIX direction
                towerYaxis2.setPower(1);
            }else if(gamepad1.dpad_down){
                towerYaxis1.setPower(1); //FIX direction
                towerYaxis2.setPower(-1);
            }else{
                towerYaxis1.setPower(0);
                towerYaxis2.setPower(0);
            }


//            if(gamepad1.dpad_up){
//                Runnable towerThread = new Thread(()->{
//                    towerYaxis2.setTargetPosition(towerYaxis2.getCurrentPosition() + 50);
//                    towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    towerYaxis2.setPower(0.95);
//                });
//                towerThread.run();
//                towerYaxis1.setTargetPosition(towerYaxis1.getCurrentPosition() - 50); //FIX direction
//                towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                towerYaxis1.setPower(0.95);
//            } else if(gamepad1.dpad_down){
//                towerYaxis1.setTargetPosition(towerYaxis1.getCurrentPosition() + 50); //FIX direction
//                towerYaxis2.setTargetPosition(towerYaxis2.getCurrentPosition() - 50);
//                towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                towerYaxis1.setPower(0.95);
//                towerYaxis2.setPower(0.95);
//            }else{
//                towerYaxis1.setPower(0);
//                towerYaxis2.setPower(0);
//                telemetry.addData("Left Trigger", gamepad1.left_trigger);
//            }


//            if(gamepad1.dpad_down){
//                towerYaxis1.setPower(1); //FIX direction
//                towerYaxis2.setPower(-1);
////                towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }

//            if(!gamepad1.dpad_down)
//            {
//                towerYaxis1.setPower(0);
//                towerYaxis2.setPower(0);
//            }

//            if(gamepad1.right_trigger > 0){
//                towerYaxis1.setTargetPosition(towerYaxis1.getCurrentPosition() + 200); //FIX direction
//                towerYaxis2.setTargetPosition(towerYaxis2.getCurrentPosition() - 200);
//                towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                towerYaxis1.setPower(0.95);
//                towerYaxis2.setPower(0.95);
//            }
//        else {
//        towerYaxis1.setPower(0);
//        towerYaxis2.setPower(0);
//    }
//        claw position .771666666666 open
//        claw position .67999999999 closed
           //tower one HIGH triangle -4224 medium circle -3194 low cross -1789 auto -643
           //tower two high 4147  medium 3140  low 1765  auto 656
            //double posOpen = .771666666666666;
            //double posClose = .6599999999999999;
           if(gamepad1.right_bumper){ //open
               claw.setPosition(.771666666666666);
           }

           if(gamepad1.left_bumper) { //close
               claw.setPosition(.6199999999999999);

           }
           /*double curr = 0;
           if(gamepad1.dpad_left){ // close a lil
               claw.setPosition(curr - .01);
           }
           if (gamepad1.dpad_right){ // open a lil
               claw.setPosition(curr + .01);
           }*/
           if (gamepad1.y) { //high
               towerYaxis1.setTargetPosition(-4224);
               towerYaxis2.setTargetPosition(4147);
               towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           }

           if (gamepad1.b){ //medium
               towerYaxis1.setTargetPosition(-3194);
               towerYaxis2.setTargetPosition(3140);
               towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


           }
           if (gamepad1.a){ //low
               towerYaxis1.setTargetPosition(-1789);
               towerYaxis2.setTargetPosition(1765);
               towerYaxis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               towerYaxis2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           }




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Claw Position",  claw.getPosition());
            telemetry.addData("TowerYaxis1 Position", towerYaxis1.getCurrentPosition());
            telemetry.addData("TowerYaxis2 Position", towerYaxis2.getCurrentPosition());
            telemetry.addData("DebugTimes", debugTimes);
            telemetry.addData("Color Sensor", sleeveNumber);
            telemetry.addData("red", colorSensor.red());
           telemetry.addData("blue", colorSensor.blue());
           telemetry.addData("green", colorSensor.green());
            telemetry.update();
        }

    }
//tower one HIGH -4224 medium -3194 low -1789 auto -643
//tower two high 4147  medium 3140  low 1765  auto 656



// Need code for moving arm side to side
// Need code for close and opening claw (left and right button)