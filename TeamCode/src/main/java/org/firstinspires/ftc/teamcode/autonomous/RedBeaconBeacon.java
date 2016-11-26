package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;
import org.firstinspires.ftc.robotcontroller.libs.sensor.RGB;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.libs.Robot;

/**
 *
 * Created by Jack Packham on 11/22/16
 */

@Autonomous(name="Red 2 Beacon", group="Autonmous: Red")
//@Disabled
public class RedBeaconBeacon extends LinearOpMode {
    Robot robot   = new Robot(hardwareMap);
    private ElapsedTime runtime = new ElapsedTime();


    static final double     COUNTS_PER_MOTOR_REV    = 280 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    static final double     FORWARD_SPEED = 0.9;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        //Step 1: Go fowards for a few seconds at full speed until turning point
//        robot.motorDriveLeft.setPower(FORWARD_SPEED);
//        robot.motorDriveRight.setPower(FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < 1.5) {
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
        encoderDrive(1, 64, 64, 10);
        //Step 2: Turn about 45 degrees
        robot.motorDriveLeft.setPower(TURN_SPEED);
        robot.motorDriveRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .5) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Step 3: Go forwards to first beacon
//        robot.motorDriveLeft.setPower(TURN_SPEED);
//        robot.motorDriveRight.setPower(TURN_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < .5) {
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
        encoderDrive(1, 7, 7, 10);
        //Step 4: Activate Beacons
        robot.motorDriveLeft.setPower(0);
        robot.motorDriveRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 5) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Step 5: Go forwards to second beacon
//        robot.motorDriveLeft.setPower(FORWARD_SPEED);
//        robot.motorDriveRight.setPower(FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < 2) {
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
        encoderDrive(1, 39, 39, 10);
        //Step 6: Activate beacon
        robot.motorDriveLeft.setPower(0);
        robot.motorDriveRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 5) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Step 7: Turn 285 degrees
        robot.motorDriveLeft.setPower(TURN_SPEED);
        robot.motorDriveRight.setPower(0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .5) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Step 8: Drive forward
        robot.motorDriveLeft.setPower(FORWARD_SPEED);
        robot.motorDriveRight.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .5) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Step 9: Shoot both balls
        robot.motorFlyLeft.setPower(1);
        robot.motorFlyRight.setPower(1);
        robot.motorIntakeElevator.setPower(.85);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 7) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Step 10: Drive forward and park
        robot.motorDriveLeft.setPower(FORWARD_SPEED);
        robot.motorDriveRight.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .5) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.motorDriveLeft.setPower(0);
        robot.motorDriveRight.setPower(0);



//        // Step 1:  Drive forward for 3 seconds
//        robot.leftMotor.setPower(FORWARD_SPEED);
//        robot.rightMotor.setPower(FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 2:  Spin right for 1.3 seconds
//        robot.leftMotor.setPower(TURN_SPEED);
//        robot.rightMotor.setPower(-TURN_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
//            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 3:  Drive Backwards for 1 Second
//        robot.leftMotor.setPower(-FORWARD_SPEED);
//        robot.rightMotor.setPower(-FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 4:  Stop and close the claw.
//        robot.leftMotor.setPower(0);
//        robot.rightMotor.setPower(0);
//        robot.leftClaw.setPosition(1.0);
//        robot.rightClaw.setPosition(0.0);
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);
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
            robot.motorDriveLeft.setPower(-Math.abs(speed));
            robot.motorDriveRight.setPower(Math.abs(speed));

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

