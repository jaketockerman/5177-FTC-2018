package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Red Do Not Use", group="Autonomous")
@Disabled
public class RedRightAuto extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware5177 robot   = new Hardware5177();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // 1120 for Neverest 40
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    VuforiaLocalizer vuforia;
    /* Turning */
    private int error = 2;

    // State used for updating telemetry
    Orientation angles;
    float velocity;

    @Override
    public void runOpMode() {
        //vuforia crap
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATLDNXT/////AAAAGbsQcc9xWkZ4hyjfNno4bXUn1WhXe+FQb1iVBK/m+2VR4IYl11u4I8iUEbbHlGQvMfns/DuDKlmzNmk+FY90IV5C03nEI2KUM0IHdjKgMqwazCUsZnAAaCOPX6kinRiE+mFSSOf9H1VEl1FxxqR/QBLMNROrF7EoKG+/uqcTOPAem4cZjV4/GtnPb4qdtrnXnQW13fk1wrvY+5Tsur2Blz5JMXDuNPDseNORhDbJBbzxuRUKz/trM2Yv0U0CLMOSVhOKD++3c+xPUWzessX0cvf7G9fiAPtJ6/al2MJZWu459xpK5gYWrtd4ITkglJIweD8shmluOtVHpOgdXl9bjZjH7TQSz2P7QuVDaBch7DT0";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.jewelHitter.setPosition(-.9);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());

        telemetry.update();


        // Transition to teleop
        //AutoTransitioner.transitionOnStop(this, "Teleop");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        robot.leftGrabber.setPosition(1);
        robot.rightGrabber.setPosition(0);
        sleep(1000);
        robot.jewelHitter.setPosition(-.25);
        sleep(1000);



        if (robot.jewelSensor.red()>robot.jewelSensor.blue()){
            encoderDrive(1,-3,-3,5);
            sleep(1000);
            robot.jewelHitter.setPosition(-.9);
            sleep(1000);
            encoderDrive(1,5,5,5);

        }
        else if (robot.jewelSensor.blue()>robot.jewelSensor.red()){
            encoderDrive(1,3,3,5);
            sleep(1000);
            robot.jewelHitter.setPosition(-.9);
            sleep(1000);
        }
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        telemetry.addData("Vuforia key", vuMark);
        if (vuMark == RelicRecoveryVuMark.RIGHT){
            encoderDrive(.7,22,22,5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(.7,-23,23,5);
            encoderDrive(.7,5,5,5);
            encoderDrive(.7,23,-23,5);
            encoderDrive(.7, 21, 21, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            robot.leftGrabber.setPosition(.5);
            robot.rightGrabber.setPosition(.5);
            sleep(1000);
            encoderDrive(.7,-8,-8,5);
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER){
            encoderDrive(.7,22,22,5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(.7,-23,23,5);
            encoderDrive(.7,10,10,5);
            encoderDrive(.7,23,-23,5);
            encoderDrive(.7, 21, 21, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            robot.leftGrabber.setPosition(.5);
            robot.rightGrabber.setPosition(.5);
            sleep(1000);
            encoderDrive(.7,-8,-8,5);
        }
        else {
            encoderDrive(.7,22,22,5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(.7,-23,23,5);
            encoderDrive(.7,17,17,5);
            encoderDrive(.7,23,-23,5);
            encoderDrive(.7, 21, 21, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            robot.leftGrabber.setPosition(.5);
            robot.rightGrabber.setPosition(.5);
            sleep(1000);
            encoderDrive(.7,-8,-8,5);
        }


        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    void turnAngle(double angle) {
        long start_time = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start_time < 5000) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            velocity = robot.imu.getAngularVelocity().zRotationRate;


            //checks angle, if the angle is a large distance from the target angle moves fast, if its close,
            //moves slowly, and if its within a small range it will stop
            double abs_angle = angles.firstAngle;
            if (abs_angle < 0) {
                abs_angle += 360.0;
            }

            if (angle <= 0) {
                angle = (angle % 360) + 360;
            } else {
                angle = angle % 360;
            }

            double oppositeAngle = (abs_angle + 180.0) % 360.0;
            double direction;
            double distance;

            //if the robot is close to the desired angle just stop moving
            //special case if the desired angle is around 360 or 0
            if ((angle == 360 || angle == 0) && (abs_angle < error || abs_angle > 360 - error)) {
                direction = 0;
                distance = error;
            } else if ((360 - angle < error) && ((abs_angle < (angle + error - 360) || abs_angle < 360) && abs_angle > (angle - error))) {
                direction = 0;
                distance = error;
            } else if ((angle < error) && (abs_angle < (angle + error) && (abs_angle > (angle - error + 360) || abs_angle > 0))) {
                direction = 0;
                distance = error;
            } else if (abs_angle < (angle + error) && abs_angle > (angle - error)) {
                direction = 0;
                distance = error;
            }
            //since the angle is cyclic and wraps at 0/360 degrees, this will make sure
            //the robot turns in the direction that will result in the smallest amount
            //of turning and check how far the robot is from the desired angle
            else if (angle == 360 || angle == 0) {
                if (abs_angle < 360 && abs_angle > 180) {
                    direction = 1.0;
                    distance = Math.abs(360 - abs_angle);
                } else {
                    direction = -1.0;
                    distance = abs_angle;
                }
            } else if (angle < 180) {
                if (abs_angle > angle && abs_angle < oppositeAngle) {
                    direction = -1.0;
                    distance = Math.abs(angle - abs_angle);
                } else {
                    direction = 1.0;
                    if (abs_angle < angle) {
                        distance = Math.abs(angle - abs_angle);
                    } else {
                        distance = Math.abs((360 + angle) - abs_angle);
                    }
                }
            } else {
                if (abs_angle < angle && abs_angle > oppositeAngle) {
                    direction = 1.0;
                    distance = Math.abs(angle - abs_angle);
                } else {
                    direction = -1.0;
                    if (abs_angle < angle) {
                        distance = Math.abs(angle - (360 + abs_angle));
                    } else {
                        distance = Math.abs(angle - abs_angle);
                    }
                }
            }

            //the (distance*0.007) term is scaled based on the distance from the desired angle
            //the robot currently is, so when it gets closer it will slow down. This is the 'P'
            // part of the PID controller.

            //Ideally we would have another term for the 'I' part of the PID controller which
            //would scale based on the integral of the distance from the desired angle. This would
            //make up for any systemic error, e.g. if the robot stalls in which case the 'I' term
            //would increase until it overcomes the stall, but integration is hard... Instead
            //the 0.16 term is there and should prevent stalling since the robot only stalls under
            //0.17 power.

            //the (velocity*01) term is scaled based on how fast the robot is going, so if it
            //is turning quickly, the velocity term will decrease the power so that it doesn't
            //overshoot. This is the 'D' part of the PID controller.

            //The direction variable is used to decide which direction to turn, if the angle
            //is close direction = 0 so the robot will stop moving.

            //increase Kp -> decreased time, increased overshoot
            //increase Ki -> decreased time, increased overshoot, increase settling time
            //increase Kd -> decreased overshoot, decreased settling time
            telemetry.addData("angle: ", abs_angle);
            telemetry.addData("goal angle: ", angle);
            telemetry.addData("angular velocity", velocity);

            robot.leftMotor.setPower(direction * ((distance * 0.0015) - (velocity * 0.002) + 0.19));
            robot.rightMotor.setPower(direction * (-((distance * 0.0015) - (velocity * 0.002) + 0.19)));

            telemetry.update();
        }
    }
}
