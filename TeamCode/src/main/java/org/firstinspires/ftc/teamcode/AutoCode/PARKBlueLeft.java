package org.firstinspires.ftc.teamcode.AutoCode;

//-----imports-----
//main imports
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//opencv imports
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//webcam imports


@Autonomous(name="PARKBlueLeft Auto", group="CompetitionAuto")
//@Disabled
public class PARKBlueLeft extends AutoSupplies {
    @Override
    public void runOpMode() {
        int daWay = 0;
        //  Establish all hardware
        initForAutonomous();
        //  Wait until start
        //waitForStart();
        initVision();
        //setArmEncoderMode();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        int path = 0;

        Recognition cone = null;
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
        long halfSec = 500;
        runtime.reset();
        while (runtime.milliseconds() <= halfSec) {
            cone = getConeType();
            if (cone != null) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                break;
            }
        }
        path = getZone(cone);
        telemetry.addLine("Path: " + path);
        telemetry.update();

        sleep(300);
        if (path == 1) {
            daWay = 1;
        } else if (path == 2) {
            daWay = 2;
        } else if(path == 3){
            daWay = 3;
        } else{ lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED); }


        sleep(300);
        if (daWay == 1) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            encoderMove(400, -1, 0);
            pause(800);
            encoderMove(625, 0, 0.6);

        } else if (daWay == 2) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            encoderMove(140, -0.35, 0.35);
            pause(800);
            turnToS(0,0.5,2);
            pause(800);
            encoderMove(700, 0, 0.85);
            pause(800);

        } else if(daWay == 3){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            encoderMove(356, 1, 0);
            pause(800);
            encoderMove(861, 0, 0.6);
            pause(800);

        } else{
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            encoderMove(352, 1, 0);
            pause(800);
            encoderMove(872, 0, 0.6);
            pause(800);
            turnToS(46,0.5,2);
            ArmUP();
            setArmEncoderMode();
            telemetry.addData("LEFT Arm Vals :: ", liftLeft.getCurrentPosition());
            telemetry.addData("RIGHT Arm Vals :: ", liftRight.getCurrentPosition());
            telemetry.update();
            //pause(2000);
            encoderMove(175, 0, 0.3);
            pause(1000);
            Release();
            pause(800);
            telemetry.addData("DOWN TIME:: ", 67);
            telemetry.update();
            encoderMove(200, 0, -0.3);
            Close();
            ArmDOWN();
            pause(2000);
            turnToS(86,0.5,3);
            telemetry.addData("After break", 36);
            telemetry.update();

        }

        if(daWay == 0) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE);
        }


    }
}