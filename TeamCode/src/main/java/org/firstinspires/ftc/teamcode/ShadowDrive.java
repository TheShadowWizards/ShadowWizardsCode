package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "ShadowDrive", group = "OpMode")
public class ShadowDrive extends LinearOpMode {

    private DcMotor Fleft = null;
    private DcMotor Bleft = null;
    private DcMotor Fright = null;
    private DcMotor Bright = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        Fleft = hardwareMap.get(DcMotor.class, "Fleft");
        Bleft = hardwareMap.get(DcMotor.class, "Bleft");
        Fright = hardwareMap.get(DcMotor.class, "Fright");
        Bright = hardwareMap.get(DcMotor.class, "Bright");

        // Set motor directions
        Fleft.setDirection(DcMotor.Direction.REVERSE);
        Bleft.setDirection(DcMotor.Direction.REVERSE);
        Fright.setDirection(DcMotor.Direction.FORWARD);
        Bright.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Example Autonomous Movement: Drive forward for 1 second
        driveForward(0.5, 1000); // 50% power for 1 second

        // Stop all motors
        stopDriving();

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }

    // Method to drive forward
    private void driveForward(double power, int durationMs) {
        Fleft.setPower(power);
        Fright.setPower(power);
        Bleft.setPower(power);
        Bright.setPower(power);

        sleep(durationMs);
    }

    // Method to stop all motors
    private void stopDriving() {
        Fleft.setPower(0);
        Fright.setPower(0);
        Bleft.setPower(0);
        Bright.setPower(0);
    }
}
