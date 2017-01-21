package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;
import org.firstinspires.ftc.teamcode.libs.Robot;


@Autonomous(name="TestGyro2", group="Autonmous")
public class TestGyro2 extends LinearOpMode {

    private Robot robot;

    private ElapsedTime runtime = new ElapsedTime();

    private MotorFunctions motorFunctions;


    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 40;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Simple Ready to run");    //
        telemetry.update();
        telemetry.addData("Status", "Initialized");

        /**
         * Initializes the library functions
         * Robot hardware and motor functions
         */
        robot = new Robot(hardwareMap);
        motorFunctions = new MotorFunctions(-1, 1, 0, 1, .05);
        //calibrasting a gyroscope
        robot.gyro.calibrate();
        sleep(5000);
        //servo wheels are flipped in configuration file
        robot.servoLeftWheel.setPosition(.45);
        robot.servoRightWheel.setPosition(.25);

        robot.servoLeftArm.setPosition(0);
        robot.servoRightArm.setPosition(.6);

        robot.servoFlyAngle.setPosition(1);

        robot.servoElbow.setPosition(0.95);
        robot.servoShoulder.setPosition(0.1);

        robot.servoFeed.setPosition(.51);

        telemetry.addData("Servos: ", "Initialized");
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Gyro: ", (int)robot.gyro.getHeading());
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Driving");
        telemetry.update();
        //encoderDrive(1.0, 27, 27, 1);
        //robot.motorDriveRight.setPower(1.0);
        //robot.motorDriveLeft.setPower(-1.0);


        tankGyroTurn(-90, 0.5);
        encoderDrive(1.0, 5, 5, 1);


        /*if (robot.leftSonar.getUltrasonicLevel() > 7.62) {
            tankGyroTurn(-14, 0.5);
            encoderDrive(0.3, 1, 1, 1);
            tankGyroTurn(14, 0.5);

        }*/

        pressTheButton();
        pressTheButton();

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorDriveLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorDriveRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
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
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
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

    private boolean goalReached(int degrees) {
        int gTolerance = degrees + 4;
        int lTolerance = degrees - 4;
        if ((int)robot.gyro.getHeading() >= gTolerance && (int)robot.gyro.getHeading() <= degrees) {
            return true;
        }
        return false;
    }

    public int heading(){//just shorthand for our Integrated Z value.
        return robot.gyro.getIntegratedZValue();
    }
    public void tankGyroTurn(int degrees, double power) { //tested

        robot.gyro.resetZAxisIntegrator();               //Reset Gyro
        float direction = Math.signum(degrees); //get +/- sign of target

        robot.motorDriveRight.setPower(direction * power);
        robot.motorDriveLeft.setPower(-direction * power);
        //move in the right direction
        //we start moving BEFORE the while loop.
        while ( Math.abs(heading()) < Math.abs(degrees)){
            //if we move IN the while loop, the app crashes.
            // we need to have something inside the loop.
            //possibly needs waitOnHardwareCycle() implemented somehow.
            telemetry.addData("Status","Turning");
            //do some BS telemetry to avoid a blank loop.
        }
        robot.motorDriveRight.setPower(0);
        robot.motorDriveLeft.setPower(0);

    }

    public void pressTheButton() {
        boolean found = false;
        Servo leftArm = robot.servoLeftArm;
        while (opModeIsActive() && !found) {
            robot.motorDriveRight.setPower(1);
            robot.motorDriveLeft.setPower(1);
            if (robot.lineDetector.getLightDetected() > 0.8) {
                robot.motorDriveRight.setPower(0);
                robot.motorDriveLeft.setPower(0);
                found = true;
            }
            idle();
        }

        if (robot.colorLeft.red() >= 40) {

            leftArm.setPosition(leftArm.getPosition() + 90);
            sleep(1000);
            leftArm.setPosition(leftArm.getPosition() - 90);

        } else {
            robot.motorDriveRight.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.motorDriveLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            encoderDrive(0.5, 5.5, 5.5, 1);
            robot.motorDriveRight.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.motorDriveLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            leftArm.setPosition(leftArm.getPosition() + 90);
            sleep(1000);
            leftArm.setPosition(leftArm.getPosition() - 90);
        }

    }

}
