package org.firstinspires.ftc.teamcode.TeleOp;


import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="LuckyTeleOp", group="TeleOp")
//@Disabled
public class LuckyTeleOp extends LinearOpMode{

    protected DcMotor motorFwdLeft = null;
    protected DcMotor motorFwdRight = null;
    protected DcMotor motorBackLeft = null;
    protected DcMotor motorBackRight = null;

    protected DcMotor liftLeft   = null;
    protected DcMotor liftRight   = null;

    BNO055IMU imu;

    public Servo intakeServo = null;
    protected RevBlinkinLedDriver lights;

    private DistanceSensor coneProx;
    private ColorSensor colorBoi;

    private DigitalChannel limitLeft;
    private DigitalChannel limitRight;


    private final ElapsedTime runtime = new ElapsedTime();

    //Encoder Values
    // Neverest 40 motor spec: quadrature encoder, 280 pulses per revolution, count = 280 *4
    private static final double COUNTS_PER_MOTOR_REV = 1120; // Neverest 40 motor encoder All drive motor gearing's
    private static final double DRIVE_GEAR_REDUCTION1 = .5; // This is < 1.0 if geared UP
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION1) / 360;

    double maxPower;
    double powerLim = 0.75;
    double moveDir = 1;
    double armPowerLim = 1;
    Orientation angles;

    // Servo values \\
    double lockServoPos = 0.0;
    double releaseServoPos = 0.4;
    double closeServoMeric = 0.05;
    double openServoMeric = 0.5;
    boolean servoModeMeric = false;
    boolean servoOpen = false;  // Only used with Meric's intake (toggle controls)
    double toggleCounter = 0.0;

    double color1 = 0;
    double blue = 0;
    double green = 0;
    double red = 0;

    /*This function determines the number of ticks a motor
    would need to move in order to achieve a certain degree*/
    private int getCountsPerDegree(double degrees, int motorNumber){
        int ans;
        if(motorNumber == 1){
            ans = (int)(degrees * COUNTS_PER_DEGREE1);
        }
        else{
            return 1;
        }
        return ans;
    }

    public void resetLiftEncoders()
    {
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // you need to change the setMode to RUN_TO_POSITION when you want to change level \\
    }

    public void hardwareSetup()
    {
        // Prepares all the hardware
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");
        liftRight.setDirection(DcMotor.Direction.FORWARD);

        // Sensors on face of robot beneath intake
        coneProx = hardwareMap.get(DistanceSensor.class, "coneProx");
        colorBoi = hardwareMap.get(ColorSensor.class, "colorBoi");
        limitLeft = hardwareMap.get(DigitalChannel.class, "limitLeft");
        limitRight = hardwareMap.get(DigitalChannel.class, "limitRight");

        resetLiftEncoders();

        motorFwdRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFwdLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Gyroscope
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        intakeServo = hardwareMap.get(Servo.class,"intakeServo");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        hardwareSetup();

        // Variables
        boolean changed1 = false;
        boolean changed2 = false;
        boolean changed3 = false;
        boolean changed4 = false;

        // Starting pos
        if (servoModeMeric) {
            intakeServo.setPosition(openServoMeric);
            servoOpen = true;
        } else {
            intakeServo.setPosition(lockServoPos);
            servoOpen = false;
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //movement controls
            //collects input from the joysticks
            double fwdBackPower = -this.gamepad1.left_stick_y * moveDir;
            double strafePower = this.gamepad1.left_stick_x * moveDir;
            double turnPower = this.gamepad1.right_stick_x;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);

            //does math to figure the power that should be applied to every motor
            double leftFrontPower = (fwdBackPower + turnPower + strafePower)*powerLim;
            double rightFrontPower = (fwdBackPower - turnPower - strafePower)*powerLim;
            double leftBackPower = (fwdBackPower + turnPower - strafePower)*powerLim;
            double rightBackPower = (fwdBackPower - turnPower + strafePower)*powerLim;

            double liftPower = -gamepad2.left_stick_y * armPowerLim;

            maxPower = Math.abs(leftFrontPower);
            if (Math.abs(rightFrontPower) > maxPower) {
                maxPower = Math.abs(rightFrontPower);
            }
            if (Math.abs(leftBackPower) > maxPower) {
                maxPower = Math.abs(leftBackPower);
            }
            if (Math.abs(rightBackPower) > maxPower) {
                maxPower = Math.abs(rightBackPower);
            }
            if (maxPower > 1) {
                leftFrontPower = leftFrontPower / maxPower;
                rightFrontPower = rightFrontPower / maxPower;
                leftBackPower = leftBackPower / maxPower;
                rightBackPower = rightBackPower / maxPower;
            }

            // Setting movement for lift and drive motors
            double max = 1.0;
            motorFwdLeft.setPower(leftFrontPower* max);
            motorFwdRight.setPower(rightFrontPower* max);
            motorBackLeft.setPower(leftBackPower* max);
            motorBackRight.setPower(rightBackPower* max);

            liftLeft.setPower(liftPower);
            liftRight.setPower(liftPower);


            // ### INTAKE SERVO CODE ### \\

            // This will change the mode if the bumper is held long enough
            if (gamepad2.left_bumper) {
                toggleCounter += 0.01;  // How long to hold down basically
                if (toggleCounter >= 1) {
                    servoModeMeric = !servoModeMeric;  //   <<<< change mode
                    toggleCounter = 0;
                }
            } else {
                toggleCounter = 0;
            }

            // Intake controls for Meric's intake design
            if (servoModeMeric) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                // Toggle opening and closing
                if (gamepad2.y && !changed4) {
                    servoOpen = !servoOpen;
                    changed4 = true;
                } else if (!gamepad2.y){
                    changed4 = false;
                }
                // Set servo pos
                if (servoOpen) {
                    intakeServo.setPosition(openServoMeric);
                } else {
                    intakeServo.setPosition(closeServoMeric);
                }
            }

            // Intake controls for original ratchet intake
            else {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                // Only release ratchet while y being pressed
                if (gamepad2.y) {
                    intakeServo.setPosition(releaseServoPos);
                } else {
                    intakeServo.setPosition(lockServoPos);
                }
            }


            // ### SPEED CHANGE STUFFINGS ### \\

            // Speed change toggle
            if(gamepad1.b && !changed1) {
                if(powerLim != 1){
                    powerLim = 1;
                    // lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                }
                else{
                    powerLim = .75;
                    // lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                }
                changed1 = true;
            } else if(!gamepad1.b){changed1 = false;}

            // Direction change toggle
            if(gamepad1.a && !changed2) {
                moveDir *= -1;
                changed2 = true;
            } else if(!gamepad1.a){changed2 = false;}


            // ### LIFT CONTROLS ### \\

            //speed limiter toggle
            if(gamepad2.b && !changed3) {
                if(armPowerLim != 1){
                    armPowerLim = 1;
                }
                else{
                    armPowerLim = .75;
                }
                changed3 = true;
            } else if(!gamepad2.b){changed3 = false;}

            // This limits the distance the lift can travel
            if(liftLeft.getCurrentPosition() > 2800 || liftRight.getCurrentPosition() > 2800){
                powerLim = 0.25;
            }
            else if(liftLeft.getCurrentPosition() > 1650 || liftRight.getCurrentPosition() > 1650){
                powerLim = .5;
            } else{
                powerLim = .75;
            }

            // ### COLOR STUFF ### \\

            color1 = colorBoi.alpha();
            red = colorBoi.red();
            green = colorBoi.green();
            blue = colorBoi.blue();


            // ### TELEMETRY STUFF (wait really?!) ### \\

            telemetry.addData("Wheel Position", motorFwdLeft.getCurrentPosition()); //to be used when the encoders are ready
            telemetry.addData("Max Speed",powerLim);
            telemetry.addData("Direction",moveDir);
            telemetry.addData("leftFrontPower :: ",leftFrontPower);
            telemetry.addData("rightFrontPower :: ",rightFrontPower);
            telemetry.addData("leftBackPower :: ",leftBackPower);
            telemetry.addData("rightBackPower :: ",rightBackPower);
            telemetry.addData("leftArm Encoder Val :: ",liftLeft.getCurrentPosition());
            telemetry.addData("rightArm Encoder Val :: ",liftRight.getCurrentPosition());
            telemetry.addData("IMU YAW :: ", imu.getAngularOrientation());
            telemetry.addData("Angle Val :: ", angles);
            telemetry.addData("range", String.format("%.01f cm", coneProx.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", color1);
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);

            telemetry.update();
        }

    }

}