package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.Robot;

/**
 * Created by Natalie on 11/6/16.
 */

@Autonomous(name="Red 2 Beacon", group="Autonmous: Red")
public class RedBeaconBeacon extends LinearOpMode {

    private Robot robot;

    private ElapsedTime runtime = new ElapsedTime();


    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        robot.servoLeftWheel.setPosition(35);
        robot.servoRightWheel.setPosition(75);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "2 Beacon Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.motorFlyLeft.setPower(.9);
        robot.motorFlyRight.setPower(1);
        sleep(250);
        robot.motorIntakeElevator.setPower(1);
        robot.servoFeed.setPosition(-1);
        telemetry.addData("Status", "Shooting");    //
        telemetry.update();
        sleep(4500);
        robot.motorFlyLeft.setPower(0);
        robot.motorFlyRight.setPower(0);
        robot.motorIntakeElevator.setPower(0);
        robot.servoFeed.setPosition(.1);
        sleep(250);

        telemetry.addData("Status", "Driving");
        telemetry.update();
        encoderDrive(1.0, 25, 25, 1);

        robot.motorDriveLeft.setPower(1);
        robot.motorDriveRight.setPower(-1);
        sleep(150); //encoderTurn


        robot.motorDriveLeft.setPower(0);
        robot.motorDriveRight.setPower(0);

        telemetry.addData("Status", "85 Driving");
        telemetry.update();
        encoderDrive(1.0, 85, 85, 1);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorDriveLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorDriveRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motorDriveLeft.setTargetPosition(newLeftTarget);
            robot.motorDriveRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorDriveLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorDriveRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorDriveLeft.setPower(Math.abs(speed)); //lift side
            robot.motorDriveRight.setPower(-Math.abs(speed)); //non lift side

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorDriveLeft.isBusy() && robot.motorDriveRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motorDriveLeft.getCurrentPosition(),
                        robot.motorDriveRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorDriveLeft.setPower(0);
            robot.motorDriveRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
