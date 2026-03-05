package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ShadowRed", group = "Autonomous")
public class ShadowRed extends LinearOpMode {

    private DcMotor fleft = null;
    private DcMotor bleft = null;
    private DcMotor fright = null;
    private DcMotor bright = null;
    private DcMotorEx rshooter = null;
    private DcMotorEx lshooter = null;
    private DcMotor intake = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        fleft = hardwareMap.get(DcMotor.class, "fleft");
        bleft = hardwareMap.get(DcMotor.class, "bleft");
        fright = hardwareMap.get(DcMotor.class, "fright");
        bright = hardwareMap.get(DcMotor.class, "bright");
        rshooter = hardwareMap.get(DcMotorEx.class, "rshooter");
        lshooter = hardwareMap.get(DcMotorEx.class, "lshooter");
        intake = hardwareMap.get(DcMotor.class, "intake");


        // Set motor directions
        fleft.setDirection(DcMotor.Direction.FORWARD);
        bleft.setDirection(DcMotor.Direction.FORWARD);
        fright.setDirection(DcMotor.Direction.REVERSE);
        bright.setDirection(DcMotor.Direction.REVERSE);
        rshooter.setDirection(DcMotorEx.Direction.FORWARD);
        lshooter.setDirection(DcMotorEx.Direction.REVERSE);
        lshooter.setVelocityPIDFCoefficients(600, 0, 0, 0);
        intake.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //driveMove(5,1);
        runForTime(1);
        // Example Autonomous Movement: Drive forward for 1 second
        //runForTime();

        // Stop all motors
        stopDriving();

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }

    // Method to drive forward
    private void driveMove(double inches, double power) {

//        fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fleft.setTargetPosition(fleft.getCurrentPosition() + 2000);
        bleft.setTargetPosition(bleft.getCurrentPosition() - 2000);
        fright.setTargetPosition(fright.getCurrentPosition() - 2000);
        bright.setTargetPosition(bright.getCurrentPosition() + 2000);
//        fleft.setTargetPosition(inches);
//        bleft.setTargetPosition(inches);
//        fright.setTargetPosition(inches);
//        bright.setTargetPosition(inches);
        fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bleft.setPower(power);
        fright.setPower(power);
        bright.setPower(power);
        fleft.setPower(power);

//        fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        sleep(durationMs);
    }

    // Method to stop all motors
    private void stopDriving() {
        fleft.setPower(0);
        fright.setPower(0);
        bleft.setPower(0);
        bright.setPower(0);
    }
//    private void ballPush() {
//        // Let flywheels spin up
//        shooter.setPower(-.33);
//        shooter.setPower(-.33);
//        sleep(5000);
//
//        // Push first ball
//        intake1.setPower(-1.0);
//        intake2.setPower(0);
//        lshooter.setPower(-.3275);
//        rshooter.setPower(-.3275);
//        sleep(300);
//        intake1.setPower(0);
//        intake2.setPower(0);
//        shooter1.setPower(-.3275);
//        shooter2.setPower(-.3275);
//
//        // Wait for flywheel to recover
//        sleep(3000);
//
//        // Push second ball
//        intake1.setPower(-1.0);
//        intake2.setPower(-1.0);
//        sleep(600);
//        intake1.setPower(0);
//        intake2.setPower(0);
//    }
    ////

    public void runForTime(double power) {
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        ElapsedTime secondTimer = new ElapsedTime();
//        secondTimer.reset();

        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rshooter.setVelocity(1550);
        lshooter.setPower(lshooter.getPower());
        sleep(4800);
        intake.setPower(.5);
        sleep(500);
        intake.setPower(0);
        sleep(1000);
        intake.setPower(1);
        sleep(700);
        intake.setPower(0);
        sleep(1000);
        intake.setPower(1);
        sleep(2000);
        intake.setPower(0);
        sleep(1000);
        driveMove(10,1);
        sleep(1000);
        //stopDriving();
        //while (opModeIsActive() && timer.seconds() < seconds1) {
//            shooter1.setPower(-.3275);
//            shooter2.setPower(-.3275);
        //driveForward(power);
//            shooter1.setPower(-.3275);
//            shooter2.setPower(-.3275);
        //}

        //stopDriving();
        //while (opModeIsActive() && secondTimer.seconds() < seconds2) {
//            shooter1.setPower(-.3275);
//            shooter2.setPower(-.3275);
        // ballPush();
        //}
    }

}
