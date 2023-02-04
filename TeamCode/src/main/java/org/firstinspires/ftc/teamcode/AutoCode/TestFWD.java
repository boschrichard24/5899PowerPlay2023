package org.firstinspires.ftc.teamcode.AutoCode;
//-----imports-----
//main imports
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//opencv imports
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//webcam imports


@Autonomous(name="TestFWD", group="CompetitionAuto")
//@Disabled
public class TestFWD extends AutoSupplies{
    //Encoder Values
    //Neverest 40 motor spec: quadrature encoder, 7 pulses per revolution, count = 7 * 40
    private static final double COUNTS_PER_MOTOR_REV = 420; // Neverest 40 motor encoder - orginal val = 280
    private static final double DRIVE_GEAR_REDUCTION = 1; // This is < 1 if geared up
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;

    public void initHardware()
    {
        //Prepares all the hardware
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorFwdRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFwdLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");
        liftRight.setDirection(DcMotor.Direction.FORWARD);

        motorFwdRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFwdLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public void initAUTOhardware()
    {
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();

        gyroParameters.mode                = BNO055IMU.SensorMode.IMU;
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.loggingEnabled      = false;
// Connect Motors to Phone \\
        //initialize hardware
        //main motors
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorFwdRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorFwdLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");
        liftRight.setDirection(DcMotor.Direction.FORWARD);
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        intakeServo = hardwareMap.get(Servo.class,"intakeServo");


        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//sensors
        /*
        distanceFwdLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceFwdLeft");
        distanceFwdRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceFwdRight");
        distanceBackLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceBackLeft");
        distanceBackRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceBackRight");
        distanceLeftTop = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeftTop");
        distanceLeftBottom = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeftBottom");
         */

        intakeServo.setPosition(lockServoPos);

// Set the direction for each of the motors \\
        motorFwdRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        //resetArmEncoders();

        //initializes imu and calibrates it. Prepares lift motor to land using the encoder
        // Lights turn green when it is calibrated
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);

        telemetry.clear();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void ENCODERmove(double degrees, double x, double y)
    {
        resetEncoders();
        double counts = degrees * COUNTS_PER_DEGREE1;
        double fwdBackPower = y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower + strafePower;
        double rightFrontPower = fwdBackPower + strafePower;
        double leftBackPower = fwdBackPower - strafePower;
        double rightBackPower = fwdBackPower - strafePower;
        double maxPower;
        double max = 1.0;
        double posPower = 0.2;
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
        //sets the power of the motors
        double averageEnc = (Math.abs(motorFwdLeft.getCurrentPosition())
                + Math.abs(motorFwdRight.getCurrentPosition())
                + Math.abs(motorBackLeft.getCurrentPosition())
                + Math.abs(motorBackRight.getCurrentPosition()))/4.0;
        while (opModeIsActive() && averageEnc <= counts){
            averageEnc = (Math.abs(motorFwdLeft.getCurrentPosition())
                    + Math.abs(motorFwdRight.getCurrentPosition())
                    + Math.abs(motorBackLeft.getCurrentPosition())
                    + Math.abs(motorBackRight.getCurrentPosition()))/4.0;
            if(posPower < 1 && averageEnc/counts < .6){
                posPower *= 1.1;
            }
            else if(posPower >= 1 && averageEnc/counts <.6){
                posPower = 1;
            }
            else if(averageEnc/counts >= .6 && posPower >= .25){
                posPower *= .99;
            }
            else{
                posPower = .25;
            }
            telemetry.addData("max :: ", max);
            telemetry.addData("posPower :: ", posPower);
            telemetry.update();
            motorFwdLeft.setPower(leftFrontPower*max*posPower);
            motorBackLeft.setPower(leftBackPower*max*posPower);
            motorFwdRight.setPower(rightFrontPower*max*posPower);
            motorBackRight.setPower(rightBackPower*max*posPower);
        }
        motorFwdLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public void TTencoderMove(double degrees, double x, double y){
        resetDriveEncoders();
        double counts = degrees * COUNTS_PER_DEGREE1;
        double fwdBackPower = y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower + strafePower;
        double rightFrontPower = fwdBackPower - strafePower;
        double leftBackPower = fwdBackPower - strafePower;
        double rightBackPower = fwdBackPower + strafePower;
        double maxPower;
        double max = 1.0;
        double posPower = 0.2;
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
        //sets the power of the motors
        double averageEnc = (Math.abs(motorFwdLeft.getCurrentPosition())
                + Math.abs(motorFwdRight.getCurrentPosition())
                + Math.abs(motorBackLeft.getCurrentPosition())
                + Math.abs(motorBackRight.getCurrentPosition()))/4.0;
        while (opModeIsActive() && averageEnc <= counts){
            averageEnc = (Math.abs(motorFwdLeft.getCurrentPosition())
                    + Math.abs(motorFwdRight.getCurrentPosition())
                    + Math.abs(motorBackLeft.getCurrentPosition())
                    + Math.abs(motorBackRight.getCurrentPosition()))/4.0;
            if(posPower < 1 && averageEnc/counts < .6){
                posPower *= 1.1;
            }
            else if(posPower >= 1 && averageEnc/counts <.6){
                posPower = 1;
            }
            else if(averageEnc/counts >= .6 && posPower >= .25){
                posPower *= .99;
            }
            else{
                posPower = .25;
            }
            motorFwdLeft.setPower(leftFrontPower*max*posPower);
            motorBackLeft.setPower(leftBackPower*max*posPower);
            motorFwdRight.setPower(rightFrontPower*max*posPower);
            motorBackRight.setPower(rightBackPower*max*posPower);
        }
        motorFwdLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackRight.setPower(0);
    }

    private void resetEncoders()
    {
        motorFwdLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFwdLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFwdRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFwdRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() {

        //  Establish all hardware
        //initHardware();
        initAUTOhardware();

        //  Wait until start
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        ENCODERmove(775,0,1);
        //TTencoderMove(775,0,1);
    }
}