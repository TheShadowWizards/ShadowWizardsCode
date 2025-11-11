package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ShadowAuto", group = "Autonomous")
public class ShadowAuto extends LinearOpMode {

    private DcMotor Fleft = null;
    private DcMotor Bleft = null;
    private DcMotor Fright = null;
    private DcMotor Bright = null;
    private DcMotor shooter1 = null;
    private DcMotor shooter2 = null;
    private DcMotor intake1 = null;
    private DcMotor intake2 = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        Fleft = hardwareMap.get(DcMotor.class, "Fleft");
        Bleft = hardwareMap.get(DcMotor.class, "Bleft");
        Fright = hardwareMap.get(DcMotor.class, "Fright");
        Bright = hardwareMap.get(DcMotor.class, "Bright");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");


        // Set motor directions
        Fleft.setDirection(DcMotor.Direction.REVERSE);
        Bleft.setDirection(DcMotor.Direction.REVERSE);
        Fright.setDirection(DcMotor.Direction.FORWARD);
        Bright.setDirection(DcMotor.Direction.FORWARD);
        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Example Autonomous Movement: Drive forward for 1 second
        runForTime(0.5, 2, 8.9); // 50% power for 1 second
        // Stop all motors
        stopDriving();

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }

    // Method to drive forward
    private void driveForward(double power) {
        Fleft.setPower(power);
        Fright.setPower(power);
        Bleft.setPower(power);
        Bright.setPower(power);
//
//        sleep(durationMs);
    }

    // Method to stop all motors
    private void stopDriving() {
        Fleft.setPower(0);
        Fright.setPower(0);
        Bleft.setPower(0);
        Bright.setPower(0);
    }
    private void ballPush() {
        // Let flywheels spin up
        shooter1.setPower(-.33);
        shooter2.setPower(-.33);
        sleep(5000);

        // Push first ball
        intake1.setPower(-1.0);
        intake2.setPower(0);
        shooter1.setPower(-.3275);
        shooter2.setPower(-.3275);
        sleep(300);
        intake1.setPower(0);
        intake2.setPower(0);
        shooter1.setPower(-.3275);
        shooter2.setPower(-.3275);

        // Wait for flywheel to recover
        sleep(3000);

        // Push second ball
        intake1.setPower(-1.0);
        intake2.setPower(-1.0);
        sleep(600);
        intake1.setPower(0);
        intake2.setPower(0);
    }
//

    public void runForTime(double power, double seconds1, double seconds2) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        ElapsedTime secondTimer = new ElapsedTime();
        secondTimer.reset();
        shooter1.setPower(-.3275);
        shooter2.setPower(-.3275);

        while (opModeIsActive() && timer.seconds() < seconds1) {
            shooter1.setPower(-.3275);
            shooter2.setPower(-.3275);
            driveForward(power);
            shooter1.setPower(-.3275);
            shooter2.setPower(-.3275);
        }

        stopDriving();
        while (opModeIsActive() && secondTimer.seconds() < seconds2) {
            shooter1.setPower(-.3275);
            shooter2.setPower(-.3275);
            ballPush();
        }
    }

}
