package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="LimitTIMEEE", group="TeleOp")
//@Disabled
public class LimitTIMEEE extends LinearOpMode{

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

    private RevTouchSensor limitLeft;
    private RevTouchSensor limitRight;


    private final ElapsedTime runtime = new ElapsedTime();

    //Encoder Values
    // Neverest 40 motor spec: quadrature encoder, 280 pulses per revolution, count = 280 *4
    private static final double COUNTS_PER_MOTOR_REV = 1120; // Neverest 40 motor encoder All drive motor gearings
    private static final double DRIVE_GEAR_REDUCTION1 = .5; // This is < 1.0 if geared UP
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION1) / 360;

    //Variables
    private double max = 1.0;
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

    double color1 = 0;
    double blue = 0;
    double green = 0;
    double red = 0;
    double servoPos = .5;


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

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
    }


    public void resetArmEncoders()
    {
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // you need to change the setMode to RUN_TO_POSITION when you want to change level \\
    }

    public void hardwareSetup()
    {
        //Prepares all the hardware
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");
        liftRight.setDirection(DcMotor.Direction.FORWARD);

        coneProx = hardwareMap.get(DistanceSensor.class, "coneProx");
        colorBoi = hardwareMap.get(ColorSensor.class, "colorBoi");
        limitLeft = hardwareMap.get(RevTouchSensor.class, "limitLeft");
        limitRight = hardwareMap.get(RevTouchSensor.class, "limitRight");

        resetArmEncoders();

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

    @Override
    public void runOpMode() {


        hardwareSetup();

        //variables
        boolean changed3 = false;
        boolean changed4 = false;
        boolean changed6 = false;
        boolean changed7 = false;
        boolean changed8 = false;

        intakeServo.setPosition(lockServoPos);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double liftPower = -gamepad2.left_stick_y * armPowerLim;

            liftLeft.setPower(liftPower);
            liftRight.setPower(liftPower);

            // ### LIFT CONTROLS ### \\

            //speed limiter toggle
            if(gamepad2.b && !changed6) {
                if(armPowerLim == .5){
                    armPowerLim = 1;
                }
                else{
                    armPowerLim = .5;
                }
                changed6 = true;
            } else if(!gamepad2.b){changed6 = false;}

            // If the Magnetic Limit Swtch is pressed, stop the motor
            if (limitLeft.isPressed() || limitRight.isPressed()) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE);
                telemetry.addData("TOUCHED_LEFT", limitLeft.isPressed());
                telemetry.addData("TOUCHED_RIGHT", limitRight.isPressed());
                telemetry.update();
            } else { // Otherwise, run the motor
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }


            if(liftLeft.getCurrentPosition() > 2800 || liftRight.getCurrentPosition() > 2800){
                powerLim = 0.25;
            }
            else if(liftLeft.getCurrentPosition() > 1650 || liftRight.getCurrentPosition() > 1650){
                powerLim = .5;
            } else{
                powerLim = .75;
            }



            color1 = colorBoi.alpha();
            red = colorBoi.red();
            green = colorBoi.green();
            blue = colorBoi.blue();


            telemetry.addData("leftArm Encoder Val :: ",liftLeft.getCurrentPosition());
            telemetry.addData("rightArm Encoder Val :: ",liftRight.getCurrentPosition());
            telemetry.addData("IMU YAW :: ", imu.getAngularOrientation());
            telemetry.addData("range", String.format("%.01f cm", coneProx.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", color1);
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("changed3", changed3);
            telemetry.addData("changed4", changed4);
            telemetry.addData("changed7", changed7);
            telemetry.addData("V2 - TOUCHED_LEFT", limitLeft.isPressed());
            telemetry.addData("V2 - TOUCHED_RIGHT", limitRight.isPressed());

            telemetry.update();
        }

    }

}